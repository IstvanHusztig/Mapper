#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <imgui_internal.h>

#include <GL/freeglut.h>
#include <portable-file-dialogs.h>

#include "lidar_odometry_utils.h"
#include "lidar_odometry.h"
#include "odometry_pipeline.h"
#include "console_reporting.h"
#include "folder_inputs.h"
#include "folder_result_sink.h"
#include <export_laz.h>

#include <utils.hpp>

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <HDMapping/Version.hpp>
#include <pfd_wrapper.hpp>
#include <ctime>
#include <chrono>
#include "toml_config_repository.h"

#ifdef _WIN32
#include <windows.h>
#include "resource.h"
#endif

///////////////////////////////////////////////////////////////////////////////////

// This is LiDAR odometry (step 1)
// This program calculates trajectory based on IMU and LiDAR data provided by MANDEYE mobile mapping system https://github.com/JanuszBedkowski/mandeye_controller
// The output is a session proving trajekctory and point clouds that can be  further processed by "multi_view_tls_registration" program.

std::string winTitle = std::string("Step 1 (Lidar odometry) ") + HDMAPPING_VERSION_STRING;

std::vector<std::string> infoLines = {
    "This program is first step in MANDEYE process.",
    "",
    "It results trajectory and point clouds as single session for 'multi_view_tls_registration_step_2' program.",
    "",
    "Next step will be to load session.json file with 'multi_view_tls_registration_step_2' program."};

// App specific shortcuts (Type and Shortcut are just for easy reference)
static const std::vector<ShortcutEntry> appShortcuts = {
    {"Normal keys", "A", ""},
    {"", "Ctrl+A", ""},
    {"", "B", ""},
    {"", "Ctrl+B", ""},
    {"", "C", ""},
    {"", "Ctrl+C", ""},
    {"", "D", ""},
    {"", "Ctrl+D", ""},
    {"", "E", ""},
    {"", "Ctrl+E", ""},
    {"", "F", ""},
    {"", "Ctrl+F", ""},
    {"", "G", ""},
    {"", "Ctrl+G", ""},
    {"", "H", ""},
    {"", "Ctrl+H", ""},
    {"", "I", ""},
    {"", "Ctrl+I", ""},
    {"", "J", ""},
    {"", "Ctrl+K", ""},
    {"", "K", ""},
    {"", "Ctrl+K", ""},
    {"", "L", ""},
    {"", "Ctrl+L", ""},
    {"", "M", ""},
    {"", "Ctrl+M", ""},
    {"", "N", ""},
    {"", "Ctrl+N", ""},
    {"", "O", ""},
    {"", "Ctrl+O", "Open data"},
    {"", "P", ""},
    {"", "Ctrl+P", ""},
    {"", "Q", ""},
    {"", "Ctrl+Q", ""},
    {"", "R", ""},
    {"", "Ctrl+R", ""},
    {"", "Shift+R", ""},
    {"", "S", ""},
    {"", "Ctrl+S", ""},
    {"", "Ctrl+Shift+S", ""},
    {"", "T", ""},
    {"", "Ctrl+T", ""},
    {"", "U", ""},
    {"", "Ctrl+U", ""},
    {"", "V", ""},
    {"", "Ctrl+V", ""},
    {"", "W", ""},
    {"", "Ctrl+W", ""},
    {"", "X", ""},
    {"", "Ctrl+X", ""},
    {"", "Y", ""},
    {"", "Ctrl+Y", ""},
    {"", "Z", ""},
    {"", "Ctrl+Z", ""},
    {"", "Shift+Z", ""},
    {"", "1-9", ""},
    {"Special keys", "Up arrow", ""},
    {"", "Shift + up arrow", ""},
    {"", "Ctrl + up arrow", ""},
    {"", "Down arrow", ""},
    {"", "Shift + down arrow", ""},
    {"", "Ctrl + down arrow", ""},
    {"", "Left arrow", ""},
    {"", "Shift + left arrow", ""},
    {"", "Ctrl + left arrow", ""},
    {"", "Right arrow", ""},
    {"", "Shift + right arrow", ""},
    {"", "Ctrl + right arrow", ""},
    {"", "Pg down", ""},
    {"", "Pg up", ""},
    {"", "- key", ""},
    {"", "+ key", ""},
    {"Mouse related", "Left click + drag", ""},
    {"", "Right click + drag", "n"},
    {"", "Scroll", ""},
    {"", "Shift + scroll", ""},
    {"", "Shift + drag", ""},
    {"", "Ctrl + left click", ""},
    {"", "Ctrl + right click", ""},
    {"", "Ctrl + middle click", ""} };

namespace fs = std::filesystem;

bool show_trajectory = true;
std::string working_directory = "";

OdometryConfig params;
GuiPreferences gui_prefs;
ConsoleReporter console;  // run messages and stages on the console

// Progress of the current run for the progress window (stage, fraction done, stage start) and Pause/Resume:
// while paused, the run blocks at its next checkpoint. Stages and progress are also forwarded to the console.
class GuiProgress : public ProgressReporter
{
public:
    explicit GuiProgress(ProgressReporter &console) : console_(&console) {}

    // a new run starts with Loading, not paused
    void reset()
    {
        set_paused(false);
        start_stage(RunStage::Loading);
    }

    void stage_started(RunStage stage) override
    {
        start_stage(stage);
        console_->stage_started(stage);
    }
    void progress(std::size_t done, std::size_t total) override
    {
        fraction_.store(total > 0 ? static_cast<float>(done) / static_cast<float>(total) : 0.0f);
        console_->progress(done, total);
    }
    void checkpoint() override
    {
        std::unique_lock<std::mutex> lock(mutex_);
        resumed_.wait(lock, [this] { return !paused_; });
    }

    void set_paused(bool paused)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            paused_ = paused;
        }
        resumed_.notify_all();
    }
    bool paused()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return paused_;
    }

    RunStage stage() const { return stage_.load(); }
    float fraction() const { return fraction_.load(); }
    // seconds since the current stage started
    double stage_seconds() const
    {
        const auto start = std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(stage_start_.load()));
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    }

private:
    void start_stage(RunStage stage)
    {
        stage_start_.store(std::chrono::steady_clock::now().time_since_epoch().count());
        fraction_.store(0.0f);
        stage_.store(stage);
    }

    ProgressReporter *console_;
    std::atomic<RunStage> stage_{RunStage::Loading};
    std::atomic<float> fraction_{0.0f};
    std::atomic<std::chrono::steady_clock::rep> stage_start_{std::chrono::steady_clock::now().time_since_epoch().count()};
    std::mutex mutex_;
    std::condition_variable resumed_;
    bool paused_ = false;
};

GuiProgress gui_progress{console};
OdometryPipeline pipeline{params, console, gui_progress}; // the current run (data shown and edited by the GUI)

// where the current settings came from (shown in the Start menu and the progress window)
std::string active_parameters = "built-in defaults";
std::string run_parameters; // active_parameters when the current run started
std::atomic<bool> loRunning{false}; // the one-click flow's worker thread is running (the progress window is shown)
std::chrono::time_point<std::chrono::system_clock> loStartTime;

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

// GUI defaults: default_settings.toml next to the exe, independent of the working directory
void set_lidar_odometry_default_params(OdometryConfig &params, RunPaths &paths, GuiPreferences &gui)
{
    const fs::path file = exe_folder() / "default_settings.toml";
    if (!fs::is_regular_file(file))
    {
        std::cout << "Default settings not found: '" << file.string() << "' (using built-in defaults)" << std::endl;
        return;
    }
    TomlConfigRepository config_repository;
    if (config_repository.load(file.string(), params, paths, gui))
    {
        active_parameters = file.string();
        std::cout << "Default settings loaded from: '" << file.string() << "'" << std::endl;
    }
}

// the view background comes from the loaded settings ([misc] clear_color)
void apply_background()
{
    bg_color = ImVec4(gui_prefs.clear_color[0], gui_prefs.clear_color[1], gui_prefs.clear_color[2], gui_prefs.clear_color[3]);
}

// Start -> Load parameters: a TOML file overlaid on the current settings
void load_parameters_file()
{
    auto input_file_names = mandeye::fd::OpenFileDialog("Load parameters file", mandeye::fd::Toml_filter, ".toml");
    if (input_file_names.empty())
        return;
    try
    {
        TomlConfigRepository config_repository;
        if (config_repository.load(input_file_names[0], params, pipeline.state.paths, gui_prefs))
        {
            active_parameters = input_file_names[0];
            apply_background();
        }
        std::cout << "Parameters loaded from: " << input_file_names[0] << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error loading TOML file: " << e.what() << std::endl;
    }
}

// file name of a parameter source for display ("built-in defaults" stays as it is)
std::string parameters_label(const std::string &source)
{
    const fs::path p(source);
    return p.has_filename() && p.has_parent_path() ? p.filename().string() : source;
}

// Helper function to format time in human-readable format
std::string formatTime(double seconds)
{
    if (seconds < 0)
        return "Calculating...";

    int hours = static_cast<int>(seconds / 3600);
    int minutes = static_cast<int>((seconds - hours * 3600) / 60);
    int secs = static_cast<int>(seconds - hours * 3600 - minutes * 60);

    std::ostringstream oss;

    if (hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << hours << "h ";
    }
    if (minutes > 0 || hours > 0)
    {
        oss << std::setw(2) << std::setfill('0') << minutes << "m ";
    }

    oss << std::setw(2) << std::setfill('0') << secs << "s";

    return oss.str();
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

// Asks for the recording folder and lists its files; on the main thread (dialog, window title). Then starts a
// fresh run: nothing from a previous folder carries over (the [paths] loaded with the settings do).
// Returns false if no existing folder was picked.
bool select_input(std::string &input_folder_name, std::vector<std::string> &input_file_names)
{
    input_folder_name = mandeye::fd::SelectFolder("Select Mandeye data folder");

    std::cout << "Selected folder: '" << input_folder_name << std::endl;

    if (!fs::exists(input_folder_name))
        return false;

    std::string newTitle = winTitle + " - ..\\" + std::filesystem::path(input_folder_name).filename().string();
    glutSetWindowTitle(newTitle.c_str());

    for (const auto &entry : fs::directory_iterator(input_folder_name))
        if (entry.is_regular_file())
            input_file_names.push_back(entry.path().string());
    if (!input_file_names.empty())
        working_directory = fs::path(input_file_names[0]).parent_path().string();

    gui_progress.reset();
    pipeline = OdometryPipeline(params, console, gui_progress, make_folder_inputs(input_file_names, console), pipeline.state.paths);
    return true;
}

// loads the recording into the current run; if that fails, tells the user and closes the program
void load_input(const std::string &input_folder_name, const std::vector<std::string> &input_file_names)
{
    if (pipeline.load())
        return;

    std::string message_info = "Problem with loading data from folder '" + input_folder_name + "' (Pease check if folder exists). Program will close once You click OK!!!";
    std::cout << message_info << std::endl;
    [[maybe_unused]]
    pfd::message message(
        "Information",
        message_info.c_str(),
        pfd::choice::ok, pfd::icon::info);
    message.result();

    exit(1);
}

// odometry + post-processing (loop closure, consistency), the same stages as the CLI
void step2()
{
    pipeline.run_all();
}

// writes the run to the next free lio_result_<n> in the recording's folder
void save_results(double elapsed_seconds)
{
    fs::path outwd = get_next_result_path(working_directory);
    TomlConfigRepository config_repository;
    FolderResultSink sink(outwd, config_repository, console);
    pipeline.save(sink, gui_prefs, elapsed_seconds);
}

void progress_window()
{
    ImGui::Begin("Progress", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text(("Working directory:\n'" + working_directory +  "'").c_str());
    ImGui::Text("Parameters: %s", parameters_label(run_parameters).c_str());
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("%s", run_parameters.c_str());

    const double elapsedSeconds = std::chrono::duration<double>(std::chrono::system_clock::now() - loStartTime).count();
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

void openData()
{
    info_gui = false;

    std::string input_folder_name;
    std::vector<std::string> input_file_names;
    if (select_input(input_folder_name, input_file_names))
    {
        // the progress window is shown from here on; loading runs on the worker thread so it stays visible
        loRunning.store(true);
        loStartTime = std::chrono::system_clock::now();
        run_parameters = active_parameters;

        std::thread loThread(
            [input_folder_name, input_file_names]()
            {
                load_input(input_folder_name, input_file_names);

                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();

                step2();

                end = std::chrono::system_clock::now();
                std::chrono::duration<double> elapsed_seconds = end - start;

                save_results(elapsed_seconds.count());

                loRunning = false;

                std::ostringstream oss;
                oss << "Data saved to folder:\n'" << working_directory << "\\lio_result_*'\n"
                    << "Calculated trajectory length: "
                    << std::fixed << std::setprecision(1) << pipeline.state.total_length_of_calculated_trajectory << "[m]\n"
                    << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str();

                [[maybe_unused]]
                pfd::message message(
                    "Information",
                    oss.str(),
                    pfd::choice::ok, pfd::icon::info);
                message.result();

            });

        loThread.detach();
    }
}

// the current run's trajectory, with a cross at its last pose
void draw_run_data()
{
    if (show_trajectory)
    {
        glPointSize(3);
        glColor3f(0, 1, 1);
        glBegin(GL_POINTS);
        for (const auto &wd : pipeline.worker_data)
        {
            for (const auto &it : wd.intermediate_trajectory)
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
        }
        glEnd();
        glPointSize(1);

        glColor3f(1, 0, 0);
        glBegin(GL_LINES);
        if (pipeline.worker_data.size() > 0)
        {
            const auto &wd = pipeline.worker_data[pipeline.worker_data.size() - 1];
            if (wd.intermediate_trajectory.size() > 0)
            {
                const auto &it = wd.intermediate_trajectory[wd.intermediate_trajectory.size() - 1];

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
    if (!(loRunning && gui_progress.stage() == RunStage::Loading))
        draw_run_data();

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

    if (!loRunning)
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
                ImGui::MenuItem(("Parameters: " + parameters_label(active_parameters)).c_str());
                ImGui::EndDisabled();
                if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
                    ImGui::SetTooltip("%s", active_parameters.c_str());

                ImGui::EndMenu();
            }

            if (ImGui::BeginMenu("View"))
            {
                ImGui::MenuItem("Show trajectory", nullptr, &show_trajectory);
                ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

                ImGui::MenuItem("Lock Z", "Shift + Z", &lock_z, !is_ortho);

                ImGui::EndMenu();
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Scene view relevant parameters");

            camMenu();

            ImGui::SameLine(ImGui::GetWindowWidth() - ImGui::CalcTextSize("Info").x - ImGui::GetStyle().ItemSpacing.x * 2 - ImGui::GetStyle().FramePadding.x * 2);

            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_HeaderHovered));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImGui::GetStyleColorVec4(ImGuiCol_Header));
            if (ImGui::SmallButton("Info"))
                info_gui = !info_gui;

            ImGui::PopStyleVar(2);
            ImGui::PopStyleColor(3);

            ImGui::EndMainMenuBar();
        }
    }

    if (loRunning)
        progress_window();

    cor_window();

    if (info_gui)
    {
        infoLines[infoLines.size() - 2] = "It saves session file in " + working_directory + "\\lio_result_*";
        info_window(infoLines, appShortcuts);
    }

    if (compass_ruler)
        drawMiniCompassWithRuler();

    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

    glutSwapBuffers();
    glutPostRedisplay();
}

void on_exit(){
    // remove cache
    std::cout << "remove cache: '" << pipeline.state.paths.working_directory_cache << "' START" << std::endl;
    pipeline.clear_point_store();
    std::cout << "remove cache: '" << pipeline.state.paths.working_directory_cache << "' FINISHED" << std::endl;
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

int main(int argc, char *argv[])
{
    try
    {
        if (argc == 4) //runnning from command line
        {
            // CLI settings: built-in defaults + the given file only (independent of the working directory)
            OdometryConfig config{};
            RunPaths paths;
            GuiPreferences gui;
            TomlConfigRepository config_repository;
            if (!fs::is_regular_file(argv[2]) || !config_repository.load(argv[2], config, paths, gui))
            {
                std::cerr << "Cannot load parameters from: '" << argv[2] << "'" << std::endl;
                return 1;
            }
            std::cout << "Parameters loaded OK from: " << argv[2] << std::endl;

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();

            std::vector<std::string> input_file_names;
            for (const auto &entry : fs::directory_iterator(argv[1]))
            {
                if (!entry.is_directory())
                {
                    std::cout << entry.path() << std::endl;
                    input_file_names.push_back(entry.path().string());
                }
            }

            ConsoleReporter reporter;
            OdometryPipeline pipeline(config, reporter, reporter, make_folder_inputs(input_file_names, reporter), paths);
            if (pipeline.load())
                log_info(reporter) << "step_1_done";

            pipeline.run_all();

            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end - start;
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            std::cout << "calculations finished computation at "
                      << std::ctime(&end_time)
                      << "Elapsed time: " << formatTime(elapsed_seconds.count()).c_str() << "s\n";

            FolderResultSink sink(argv[3], config_repository, reporter);
            pipeline.save(sink, gui, elapsed_seconds.count());
        }
        else //full GUI mode
        {
            std::cout << argv[0] << " input_folder parameters(*.toml) output_folder" << std::endl;

            set_lidar_odometry_default_params(params, pipeline.state.paths, gui_prefs);
            apply_background();

             

            initGL(&argc, argv, winTitle, display, mouse);
            glutCloseFunc(on_exit);

            glutMainLoop();

            ImGui_ImplOpenGL2_Shutdown();
            ImGui_ImplGLUT_Shutdown();
            ImGui::DestroyContext();
        }
    }
    catch (const std::bad_alloc& e)
    {
        std::cerr << "System is out of memory : " << e.what() << std::endl;
        mandeye::fd::OutOfMemMessage();
    }
    catch (const std::exception& e)
    {
        std::cout << e.what();
    }
    catch (...)
    {
        std::cerr << "Unknown fatal error occurred." << std::endl;
    }

    return 0;
}