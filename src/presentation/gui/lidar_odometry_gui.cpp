#include <imgui.h>
#include <imgui_impl_glut.h>
#include <iostream>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <GL/freeglut.h>
#include <portable-file-dialogs.h>

#include "gui_view_model.h"
#include <export_laz.h>

#include <utils.hpp>

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <HDMapping/Version.hpp>
#include <pfd_wrapper.hpp>
#include <ctime>
#include <chrono>
#include "format_time.h"

#ifdef _WIN32
#include <windows.h>
#include "resource.h"
#endif

///////////////////////////////////////////////////////////////////////////////////

// dMapper: LiDAR + IMU mapping. Computes the trajectory and point cloud of a recording folder
// (lidar*.laz + imu*.csv) and saves them as a result folder (session.mjs, trajectories, LAZ point clouds).

std::string winTitle = "dMapper";

namespace fs = std::filesystem;

bool show_trajectory = true;

// the GUI's state and actions (settings, the current run); set in run_gui, because GLUT callbacks take no user data
GuiViewModel *view_model = nullptr;

// what a run reports, shown as message boxes (on the run's worker thread)
class MessageBoxNotifier : public RunNotifier
{
public:
    // tells the user and closes the program
    void load_failed(const std::string &folder) override
    {
        std::string message_info = "Problem with loading data from folder '" + folder + "' (Pease check if folder exists). Program will close once You click OK!!!";
        std::cout << message_info << std::endl;
        [[maybe_unused]]
        pfd::message message(
            "Information",
            message_info.c_str(),
            pfd::choice::ok, pfd::icon::info);
        message.result();

        exit(1);
    }

    void run_saved(const std::string &working_directory, double trajectory_length_m, double elapsed_seconds) override
    {
        std::ostringstream oss;
        oss << "Data saved to folder:\n'" << working_directory << "\\lio_result_*'\n"
            << "Calculated trajectory length: "
            << std::fixed << std::setprecision(1) << trajectory_length_m << "[m]\n"
            << "Elapsed time: " << formatTime(elapsed_seconds).c_str();

        [[maybe_unused]]
        pfd::message message(
            "Information",
            oss.str(),
            pfd::choice::ok, pfd::icon::info);
        message.result();
    }
};

// Live point cloud preview, built and drawn on the main thread only. Each finished window's stored (decimated)
// points are placed with that window's optimized trajectory and thinned to a display voxel across all windows.
// During the odometry only windows reported done are read (nothing writes them until post-processing); after
// the run the preview is rebuilt from the final, post-processed trajectory.
class PreviewCloud
{
public:
    static constexpr double voxel_m = 0.10;
    static constexpr std::size_t max_points = 3000000;
    static constexpr std::size_t windows_per_frame = 4;

    void clear()
    {
        xyz_.clear();
        rgb_.clear();
        voxels_.clear();
        next_window_ = 0;
        z_min_ = std::numeric_limits<float>::max();
        z_max_ = std::numeric_limits<float>::lowest();
        capped_ = false;
        dirty_ = true;
        compiled_at_ = {};
    }

    std::size_t next_window() const { return next_window_; }

    // adds windows next_window() .. limit-1, at most windows_per_frame of them
    void add_windows(const OdometryPipeline &run, std::size_t limit, Logger &logger)
    {
        limit = std::min(limit, run.windows.size());
        bool added = false;
        for (std::size_t n = 0; n < windows_per_frame && next_window_ < limit && !capped_; n++)
            added |= add_window(run, next_window_++, logger);
        if (added)
        {
            recolor();
            dirty_ = true;
        }
    }

    std::size_t size() const { return xyz_.size() / 3; }

    // Drawn from a display list: the points are handed to OpenGL only when the list is rebuilt (at most once per
    // second while points are added), not every frame, so the driver doesn't keep copies of the whole cloud.
    void draw()
    {
        const auto now = std::chrono::steady_clock::now();
        if (dirty_ && now - compiled_at_ >= std::chrono::seconds(1))
        {
            if (list_ == 0)
                list_ = glGenLists(1);
            glNewList(list_, GL_COMPILE);
            if (!xyz_.empty())
            {
                glEnableClientState(GL_VERTEX_ARRAY);
                glEnableClientState(GL_COLOR_ARRAY);
                glVertexPointer(3, GL_FLOAT, 0, xyz_.data());
                glColorPointer(3, GL_FLOAT, 0, rgb_.data());
                glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(xyz_.size() / 3));
                glDisableClientState(GL_COLOR_ARRAY);
                glDisableClientState(GL_VERTEX_ARRAY);
            }
            glEndList();
            dirty_ = false;
            compiled_at_ = now;
        }
        if (list_ == 0)
            return;
        glPointSize(2);
        glCallList(list_);
        glPointSize(1);
    }

private:
    bool add_window(const OdometryPipeline &run, std::size_t window, Logger &logger)
    {
        std::vector<Point3Di> points;
        if (!run.load_window_points(window, points))
            return false;
        const auto &trajectory = run.windows[window].poses;
        bool added = false;
        for (const auto &p : points)
        {
            if (p.index_pose < 0 || p.index_pose >= static_cast<int>(trajectory.size()))
                continue;
            const Eigen::Vector3d g = trajectory[p.index_pose] * p.point;
            if (!voxels_.insert(voxel_key(g)).second)
                continue;
            if (xyz_.size() / 3 >= max_points)
            {
                capped_ = true;
                log_info(logger) << "point cloud preview: limit of " << max_points << " points reached";
                break;
            }
            xyz_.push_back(static_cast<float>(g.x()));
            xyz_.push_back(static_cast<float>(g.y()));
            xyz_.push_back(static_cast<float>(g.z()));
            z_min_ = std::min(z_min_, static_cast<float>(g.z()));
            z_max_ = std::max(z_max_, static_cast<float>(g.z()));
            added = true;
        }
        return added;
    }

    static std::uint64_t voxel_key(const Eigen::Vector3d &p)
    {
        // 21 bits per axis around 0: +-104 km at 0.1 m
        auto cell = [](double v) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(v / voxel_m)) + (1ll << 20)) & 0x1FFFFF; };
        return (cell(p.x()) << 42) | (cell(p.y()) << 21) | cell(p.z());
    }

    // height ramp blue -> cyan -> green -> yellow -> red over the preview's z range
    void recolor()
    {
        const float range = (z_max_ - z_min_) > 1e-3f ? (z_max_ - z_min_) : 1.0f;
        rgb_.resize(xyz_.size());
        for (std::size_t i = 0; i < xyz_.size(); i += 3)
        {
            const float t = (xyz_[i + 2] - z_min_) / range;
            rgb_[i] = std::clamp(1.5f - std::fabs(4.0f * t - 3.0f), 0.0f, 1.0f);
            rgb_[i + 1] = std::clamp(1.5f - std::fabs(4.0f * t - 2.0f), 0.0f, 1.0f);
            rgb_[i + 2] = std::clamp(1.5f - std::fabs(4.0f * t - 1.0f), 0.0f, 1.0f);
        }
    }

    std::vector<float> xyz_;
    std::vector<float> rgb_;
    std::unordered_set<std::uint64_t> voxels_;
    std::size_t next_window_ = 0;
    float z_min_ = std::numeric_limits<float>::max();
    float z_max_ = std::numeric_limits<float>::lowest();
    bool capped_ = false;
    GLuint list_ = 0;
    bool dirty_ = false;
    std::chrono::steady_clock::time_point compiled_at_{};
};

PreviewCloud preview_cloud;
bool show_point_cloud = true;
bool preview_was_running = false; // a run was going in the previous frame: its end triggers the final rebuild
bool preview_final_pending = false;

// adds what is ready to the preview: finished windows during the odometry, all windows after the run
void update_preview()
{
    const bool running = view_model->running();
    const OdometryPipeline &run = view_model->run();
    Logger &console = view_model->console();
    if (preview_was_running && !running)
    {
        // the run has saved: rebuild from the final (post-processed) trajectory
        preview_cloud.clear();
        preview_final_pending = true;
    }
    preview_was_running = running;

    if (running)
    {
        if (view_model->progress().stage() == RunStage::Odometry)
        {
            const std::size_t before = preview_cloud.next_window();
            preview_cloud.add_windows(run, view_model->progress().windows_done(), console);
            if (before < run.windows.size() && preview_cloud.next_window() >= run.windows.size())
                log_info(console) << "point cloud preview: " << preview_cloud.size() << " points after the odometry";
        }
    }
    else if (preview_final_pending)
    {
        preview_cloud.add_windows(run, run.windows.size(), console);
        if (preview_cloud.next_window() >= run.windows.size())
        {
            preview_final_pending = false;
            log_info(console) << "point cloud preview: " << preview_cloud.size() << " points (final)";
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////

// folder of the running executable (the build copies default_settings.toml next to it)
fs::path exe_folder()
{
#ifdef _WIN32
    wchar_t buffer[MAX_PATH];
    const DWORD n = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (n > 0 && n < MAX_PATH)
        return fs::path(std::wstring(buffer, n)).parent_path();
#endif
    return fs::current_path();
}

// the view background comes from the loaded settings ([misc] clear_color)
void apply_background()
{
    const GuiPreferences &gui_prefs = view_model->preferences();
    bg_color = ImVec4(gui_prefs.clear_color[0], gui_prefs.clear_color[1], gui_prefs.clear_color[2], gui_prefs.clear_color[3]);
}

// Start -> Load parameters: a TOML file overlaid on the current settings
void load_parameters_file()
{
    auto input_file_names = mandeye::fd::OpenFileDialog("Load parameters file", mandeye::fd::Toml_filter, ".toml");
    if (input_file_names.empty())
        return;
    if (view_model->load_parameters(input_file_names[0]))
        apply_background();
}

// file name of a parameter source for display ("built-in defaults" stays as it is)
std::string parameters_label(const std::string &source)
{
    const fs::path p(source);
    return p.has_filename() && p.has_parent_path() ? p.filename().string() : source;
}

// Helper function to format estimated completion time
std::string formatCompletionTime(double remainingSeconds)
{
    if (remainingSeconds < 0)
        return "Calculating...";

    auto now = std::chrono::system_clock::now();
    auto estimatedCompletion = now + std::chrono::seconds(static_cast<long long>(remainingSeconds));
    auto completion_time_t = std::chrono::system_clock::to_time_t(estimatedCompletion);

    // Format as HH:MM:SS - Cross-platform time formatting
    struct tm completion_tm;
#ifdef _WIN32
    localtime_s(&completion_tm, &completion_time_t);
#else
    localtime_r(&completion_time_t, &completion_tm);
#endif

    char timeStr[32];
    strftime(timeStr, sizeof(timeStr), "%H:%M", &completion_tm);

    return std::string(timeStr);
}

void progress_window()
{
    ImGui::Begin("Progress", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    GuiProgress &gui_progress = view_model->progress();
    ImGui::Text(("Working directory:\n'" + view_model->working_directory() +  "'").c_str());
    ImGui::Text("Parameters: %s", parameters_label(view_model->run_parameters()).c_str());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", view_model->run_parameters().c_str());

    const double elapsedSeconds = view_model->elapsed_seconds();
    const RunStage stage = gui_progress.stage();

    char timeInfo[512];
    if (stage == RunStage::Odometry)
    {
        // percentage and estimate for the odometry only: the remaining time is extrapolated from this stage
        const float progress = gui_progress.fraction();
        char progressText[256];
        snprintf(progressText, sizeof(progressText), "%s: %.1f%%", run_stage_name(stage), progress * 100.0f);
        if (progress > 0.01f && progress < 1.0f)
        {
            const double stageSeconds = gui_progress.stage_seconds();
            const double estimatedTimeRemaining = stageSeconds / progress - stageSeconds;
            snprintf(timeInfo, sizeof(timeInfo),
                     "Elapsed: %s | Remaining: %s | Estimated finish: ~%s",
                     formatTime(elapsedSeconds).c_str(),
                     formatTime(estimatedTimeRemaining).c_str(),
                     formatCompletionTime(estimatedTimeRemaining).c_str());
        }
        else
        {
            snprintf(timeInfo, sizeof(timeInfo),
                     "Elapsed: %s | Calculating completion time...",
                     formatTime(elapsedSeconds).c_str());
        }
        ImGui::ProgressBar(progress, ImVec2(-1.0f, 0.0f), progressText);
    }
    else
    {
        ImGui::Text("%s...", run_stage_name(stage));
        snprintf(timeInfo, sizeof(timeInfo), "Elapsed: %s", formatTime(elapsedSeconds).c_str());
    }
    ImGui::Text("%s", timeInfo);

    ImGui::NewLine();

    if (!gui_progress.paused())
    {
        if (ImGui::Button("Pause"))
            gui_progress.set_paused(true);

        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("Pausing calculation will not save intermediate state!");
            ImGui::Text("You can't resume if you close the program!");
            ImGui::EndTooltip();
        }
    }
    else
    {
        if (ImGui::Button("Resume"))
            gui_progress.set_paused(false);
    }
    ImGui::SameLine();
    ImGui::Text("Also check console for progress..");

    ImGui::End();
}

// Start -> Load data and process: asks for the recording folder (main thread, file dialog) and starts the run
void openData()
{
    const std::string input_folder_name = mandeye::fd::SelectFolder("Select recording folder (lidar*.laz + imu*.csv)");
    if (view_model->start_run(input_folder_name))
    {
        // a fresh run: the preview starts empty
        preview_cloud.clear();
        preview_final_pending = false;
    }
}

// the current run's trajectory, with a cross at its last pose
void draw_run_data()
{
    if (show_point_cloud)
        preview_cloud.draw();

    if (show_trajectory)
    {
        glPointSize(3);
        glColor3f(0, 1, 1);
        glBegin(GL_POINTS);
        for (const auto &wd : view_model->run().windows)
        {
            for (const auto &it : wd.poses)
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
        }
        glEnd();
        glPointSize(1);

        glColor3f(1, 0, 0);
        glBegin(GL_LINES);
        const auto &windows = view_model->run().windows;
        if (windows.size() > 0)
        {
            const auto &wd = windows[windows.size() - 1];
            if (wd.poses.size() > 0)
            {
                const auto &it = wd.poses[wd.poses.size() - 1];

                glVertex3f(it(0, 3) - 1, it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + 1, it(1, 3), it(2, 3));

                glVertex3f(it(0, 3), it(1, 3) - 1, it(2, 3));
                glVertex3f(it(0, 3), it(1, 3) + 1, it(2, 3));

                glVertex3f(it(0, 3), it(1, 3), it(2, 3) - 1);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3) + 1);
            }
        }
        glEnd();
    }
}

// The next frame is scheduled instead of requested at once: about 10 frames per second during a run (so the
// redraw doesn't take time from the processing) and about 60 otherwise. Mouse moves still redraw at once (core).
bool redraw_scheduled = false;

void redraw_timer(int)
{
    redraw_scheduled = false;
    glutPostRedisplay();
}

void schedule_redraw()
{
    if (redraw_scheduled)
        return;
    redraw_scheduled = true;
    glutTimerFunc(view_model->running() ? 100 : 16, redraw_timer, 0);
}

void display()
{ 
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

    glClearColor(bg_color.x * bg_color.w, bg_color.y * bg_color.w, bg_color.z * bg_color.w, bg_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    updateCameraTransition();

    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);

    viewLocal = Eigen::Affine3f::Identity();

    viewLocal.translate(rotation_center);

    viewLocal.translate(Eigen::Vector3f(translate_x, translate_y, translate_z));
    if (!lock_z)
        viewLocal.rotate(Eigen::AngleAxisf(rotate_x * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
    else
        viewLocal.rotate(Eigen::AngleAxisf(-90.0 * DEG_TO_RAD, Eigen::Vector3f::UnitX()));
    viewLocal.rotate(Eigen::AngleAxisf(rotate_y * DEG_TO_RAD, Eigen::Vector3f::UnitZ()));

    viewLocal.translate(-rotation_center);

    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(viewLocal.matrix().data());

    showAxes();

    // the run's data is not drawn while it is loaded: the worker thread is still growing its vectors
    if (!(view_model->running() && view_model->progress().stage() == RunStage::Loading))
    {
        update_preview();
        draw_run_data();
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();
	ImGui::NewFrame();

    ShowMainDockSpace();

    view_kbd_shortcuts();

    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O, false))
    {
        openData();

        //workaround
        io.AddKeyEvent(ImGuiKey_O, false);
        io.AddKeyEvent(ImGuiMod_Ctrl, false);
    }

    if (!view_model->running())
    {
        if (ImGui::BeginMainMenuBar())
        {
            if (ImGui::BeginMenu("Start"))
            {
                if (ImGui::MenuItem("1. Load parameters..."))
                    load_parameters_file();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Load processing settings from a TOML file (the only way to change them)");

                if (ImGui::MenuItem("2. Load data and process...", "Ctrl+O"))
                    openData();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Select the folder with the IMU *.csv and LiDAR *.laz files; it is loaded, processed and saved to lio_result_*");

                ImGui::Separator();
                ImGui::BeginDisabled();
                ImGui::MenuItem(("Parameters: " + parameters_label(view_model->active_parameters())).c_str());
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("%s", view_model->active_parameters().c_str());

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View"))
            {
                ImGui::MenuItem("Show point cloud", nullptr, &show_point_cloud);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Live preview of the map, coloured by height (rebuilt from the final trajectory after a run)");
                ImGui::MenuItem("Show trajectory", nullptr, &show_trajectory);
                ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

                ImGui::MenuItem("Lock Z", "Shift + Z", &lock_z, !is_ortho);

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Scene view relevant parameters");

            camMenu();

            ImGui::EndMainMenuBar();
        }
    }

    if (view_model->running())
        progress_window();

    cor_window();

    if (compass_ruler)
        drawMiniCompassWithRuler();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    schedule_redraw();
}

void on_exit(){
    view_model->clear_point_store();
}

void mouse(int glut_button, int state, int x, int y)
{
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON)
        button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON)
        button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON)
        button = 2;
    if (button != -1 && state == GLUT_DOWN)
        io.MouseDown[button] = true;
    if (button != -1 && state == GLUT_UP)
        io.MouseDown[button] = false;

    static int glutMajorVersion = glutGet(GLUT_VERSION) / 10000;
    if (state == GLUT_DOWN && (glut_button == 3 || glut_button == 4) && glutMajorVersion < 3)
        wheel(glut_button, glut_button == 3 ? 1 : -1, x, y);

    if (!io.WantCaptureMouse)
    {
        if ((glut_button == GLUT_MIDDLE_BUTTON || glut_button == GLUT_RIGHT_BUTTON) && state == GLUT_DOWN && io.KeyCtrl)
            setNewRotationCenter(x, y);

        if (state == GLUT_DOWN)
            mouse_buttons |= 1 << glut_button;
        else if (state == GLUT_UP)
            mouse_buttons = 0;
        
        mouse_old_x = x;
        mouse_old_y = y;
    }
}

// the GUI front end (any argument count other than the 3 of the CLI)
int run_gui(int argc, char *argv[])
{
    std::cout << argv[0] << " input_folder parameters(*.toml) output_folder" << std::endl;

    MessageBoxNotifier notifier;
    GuiViewModel model(notifier);
    view_model = &model;

    // GUI defaults: default_settings.toml next to the exe, independent of the working directory
    view_model->load_default_parameters(exe_folder() / "default_settings.toml");
    apply_background();

     

    initGL(&argc, argv, winTitle, display, mouse);
    glutCloseFunc(on_exit);

    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();
    ImGui::DestroyContext();
    return 0;
}
