#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
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
#include <session.h>
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

bool is_settings_gui = false;
bool show_reference_buckets = true;
bool show_reference_points = false;
int dec_reference_points = 100;
bool show_initial_points = true;
bool show_trajectory = true;
bool show_trajectory_as_axes = false;
bool simple_gui = true;
bool step_1_done = false;
bool step_2_done = false;
bool calculations_failed = false;



std::string working_directory = "";
bool initial_transformation_gizmo = false;

float m_gizmo[] = {1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1};

int index_begin = -1;
int index_end = -1;
bool gizmo_stretch_interval = false;
Eigen::Affine3d stretch_gizmo_m = Eigen::Affine3d::Identity();

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
Session session;

// where the current settings came from (shown in the Start menu and the progress window)
std::string active_parameters = "built-in defaults";
std::string run_parameters; // active_parameters when the current run started
std::atomic<bool> loRunning{false}; // the one-click flow's worker thread is running (the progress window is shown)
double compute_all_seconds = 0.0; // duration of the advanced GUI's "Compute all", written with the result
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
    {
        step_1_done = true;
        return;
    }

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

void step1()
{
    std::string input_folder_name;
    std::vector<std::string> input_file_names;
    if (select_input(input_folder_name, input_file_names))
        load_input(input_folder_name, input_file_names);
}

// odometry + post-processing (loop closure, consistency), the same stages as the CLI
void step2()
{
    if (pipeline.run_all())
        step_2_done = true;
    else
        calculations_failed = true;
}

void save_results(bool info, double elapsed_seconds)
{
    fs::path outwd = get_next_result_path(working_directory);
    TomlConfigRepository config_repository;
    FolderResultSink sink(outwd, config_repository, console);
    pipeline.save(sink, gui_prefs, elapsed_seconds);
    if (info)
    {
        std::string message_info = "Results saved to folder: '" + outwd.string() + "'";
        std::cout << message_info << std::endl;
        [[maybe_unused]]
        pfd::message message(
            "Information",
            message_info.c_str(),
            pfd::choice::ok, pfd::icon::info);
        message.result();
    }
}

void settings_gui()
{
    if (ImGui::Begin("Settings", &is_settings_gui))
    {
        ImGui::Checkbox("simple_gui", &simple_gui);
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Show even more parameters");

        ImGui::NewLine();

        if (!loRunning)
        {
            ImGui::Text("motion_model_correction [deg]:");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("om, rotation via X", &params.motion_model_correction.om);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);
            ImGui::InputDouble("fi, rotation via Y", &params.motion_model_correction.fi);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);
            ImGui::InputDouble("ka, rotation via Z", &params.motion_model_correction.ka);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);
            ImGui::PopItemWidth();

            if (ImGui::Button("Set example"))
            {
                params.motion_model_correction.om = 0.0;
                params.motion_model_correction.fi = 0.05;
                params.motion_model_correction.ka = 0.0;
            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("motion_model_corrections for LiDAR X-axis (forward direction)");

            ImGui::NewLine();

            ImGui::Text("lidar_odometry_motion_model sigmas [m] / [deg]:");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("position/rotational uncertainties");

            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x_1", &params.lidar_odometry_motion_model_x_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om_1", &params.lidar_odometry_motion_model_om_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y_1", &params.lidar_odometry_motion_model_y_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi_1", &params.lidar_odometry_motion_model_fi_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z_1", &params.lidar_odometry_motion_model_z_1_sigma_m, 0.0, 0.0, "%.4f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka_1", &params.lidar_odometry_motion_model_ka_1_sigma_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::InputDouble("acc_1 [m/s^2]", &params.lidar_odometry_motion_model_acc_1_sigma_mps2, 0.0, 0.0, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Velocity-continuity prior (acceleration sigma), also across window boundaries; 0 = disabled");

            ImGui::PopItemWidth();

            ImGui::Text("lidar_odometry_motion_model_fix_origin sigmas [m] / [deg]:");

            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x_1##o", &params.lidar_odometry_motion_model_fix_origin_x_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om_1##o", &params.lidar_odometry_motion_model_fix_origin_om_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y_1##o", &params.lidar_odometry_motion_model_fix_origin_y_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi_1##o", &params.lidar_odometry_motion_model_fix_origin_fi_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z_1##o", &params.lidar_odometry_motion_model_fix_origin_z_1_sigma_m);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka_1##o", &params.lidar_odometry_motion_model_fix_origin_ka_1_sigma_deg);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();
        }

        ImGui::NewLine();

        if (!simple_gui)
        {
			ImGui::SetNextItemWidth(ImGuiNumberWidth);
            ImGui::InputInt("Threshold nr poses", &params.threshold_nr_poses);
            if (params.threshold_nr_poses < 1)
            {
                params.threshold_nr_poses = 1;
            }

            ImGui::NewLine();
        }

        if (calculations_failed)
        {
            ImGui::Text("CALCULATIONS FAILED... please read information in console");
            ImGui::End();
            return;
        }

        ImGui::PushItemWidth(ImGuiNumberWidth);
        ImGui::InputDouble("Filter threshold XY inner [m]", &params.filter_threshold_xy_inner, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("all local points inside lidar xy_circle radius will be removed during load");
            ImGui::Text("Minimum value is given by Lidar's specs (Close Proximity Blind Zone)");
            ImGui::Text("Value can be higher to filter out close range permanent obstacles");
            ImGui::Text("e.g.: 0.1[m] for Livox Mid-360");
            ImGui::EndTooltip();
        }
        ImGui::InputDouble("Filter threshold XY outer [m]", &params.filter_threshold_xy_outer, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
        {
            ImGui::BeginTooltip();
            ImGui::Text("all local points outside lidar xy_circle radius will be removed during load");
            ImGui::Text("Maximum value is given by Lidar's specs (Detection Range)");
            ImGui::Text("Value can be lower to adapt for different reflectivity or real world contrains");
            ImGui::Text("e.g.: 70[m] @ 80%% reflectivity for Livox Mid-360");
            ImGui::EndTooltip();
        }
        ImGui::InputDouble("Threshold output filter [m]", &params.threshould_output_filter, 0.0, 0.0, "%.3f");
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("all local points inside lidar xy_circle radius will be removed during save");
        ImGui::PopItemWidth();

        if (!simple_gui)
        {
            ImGui::NewLine();
            ImGui::Text("NDT bucket size (inner/outer)");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("##x", &params.in_out_params_indoor.resolution_X, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            if (params.in_out_params_indoor.resolution_X < 0.01)
            {
                params.in_out_params_indoor.resolution_X = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("X##ndt", &params.in_out_params_outdoor.resolution_X, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            if (params.in_out_params_outdoor.resolution_X < 0.01)
            {
                params.in_out_params_outdoor.resolution_X = 0.01;
            }

            ImGui::InputDouble("##y", &params.in_out_params_indoor.resolution_Y, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            if (params.in_out_params_indoor.resolution_Y < 0.01)
            {
                params.in_out_params_indoor.resolution_Y = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("Y##ndt", &params.in_out_params_outdoor.resolution_Y, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            if (params.in_out_params_outdoor.resolution_Y < 0.01)
            {
                params.in_out_params_outdoor.resolution_Y = 0.01;
            }

            ImGui::InputDouble("##z", &params.in_out_params_indoor.resolution_Z, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            if (params.in_out_params_indoor.resolution_Z < 0.01)
            {
                params.in_out_params_indoor.resolution_Z = 0.01;
            }
            ImGui::SameLine();
            ImGui::InputDouble("Z##ndt", &params.in_out_params_outdoor.resolution_Z, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            if (params.in_out_params_outdoor.resolution_Z < 0.01)
            {
                params.in_out_params_outdoor.resolution_Z = 0.01;
            }

            ImGui::NewLine();

            ImGui::InputDouble("Decimation", &params.decimation, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Larger value of decimation better performance, but worse accuracy");

            ImGui::InputDouble("Max distance of processed points [m]", &params.max_distance_lidar, 0.0, 0.0, "%.2f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Local LiDAR coordinates");

            ImGui::InputInt("Number of iterations", &params.nr_iter);
            ImGui::InputDouble("Sliding window trajectory length threshold [m]", &params.sliding_window_trajectory_length_threshold, 0.0, 0.0, "%.2f");
            ImGui::InputInt("Threshold initial points", &params.threshold_initial_points);

            ImGui::NewLine();

            static int fusionConvention; // 0=NWU, 1=ENU, 2=NED
            // initialize if none selected
            if (fusionConvention < 0 || fusionConvention > 2)
                fusionConvention = 0;

            ImGui::Text("Fusion convention: ");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Coordinate system conventions for sensor fusion defining how the axes are oriented relative to world frame");

            ImGui::SameLine();
            ImGui::RadioButton("NWU", &fusionConvention, 0);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("North West Up");
            ImGui::SameLine();
            ImGui::RadioButton("ENU", &fusionConvention, 1);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("East North Up");
            ImGui::SameLine();
            ImGui::RadioButton("NED", &fusionConvention, 2);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("North East Down");

            // then update your bools if you still need them
            params.fusionConventionNwu = (fusionConvention == 0);
            params.fusionConventionEnu = (fusionConvention == 1);
            params.fusionConventionNed = (fusionConvention == 2);

            ImGui::NewLine();

            ImGui::Checkbox("Use motion from previous step", &params.use_motion_from_previous_step);
            ImGui::InputDouble("AHRS gain", &params.ahrs_gain, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
            {
                ImGui::BeginTooltip();
                ImGui::Text("Attitude and Heading Reference System gain:");
                ImGui::Text("How strongly the accelerometer/magnetometer corrections influence the orientation estimate versus gyroscope integration");
                ImGui::Text("Larger value means faster response to changes in orientation, but more noise");
                ImGui::EndTooltip();
            }

            ImGui::PopItemWidth();
        }
        ImGui::NewLine();
        if (!step_1_done)
        {
            if (ImGui::Button("Load data"))
            {
                loStartTime = std::chrono::system_clock::now();
                
                step1();
                std::cout << "Load data done please click 'Compute all' to continue calculations" << std::endl;

                std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - loStartTime;
                double elapsedSeconds = elapsed.count();

                std::cout << "Elapsed time: " << formatTime(elapsedSeconds).c_str() << std::endl;

            }
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Select folder containing IMU *.csv and LiDAR *.laz files produced by MANDEYE (e.g.: 'continousScanning_*')");
        }
        if (step_1_done && !step_2_done)
        {
            if (ImGui::Button("Compute all"))
            {
                const auto compute_start = std::chrono::system_clock::now();
                step2();
                compute_all_seconds = std::chrono::duration<double>(std::chrono::system_clock::now() - compute_start).count();
            }

            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Press this button for automatic LiDAR odometry calculation (with loop closure and consistency when enabled) -> it will produce trajectory");

            // used by the consistency pass inside Compute all
            ImGui::Checkbox("Use multiple Gaussians for each bucket", &params.use_mutliple_gaussian);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Multiple Gaussians suppose to work better in floor plan indoor scenarios (multiple neighbouring rooms)");
        }

        if (step_1_done && step_2_done)
        {
            if (ImGui::Button("Save result"))
                save_results(true, compute_all_seconds);

            ImGui::SameLine();
            ImGui::Text("Press this button for saving resulting trajectory and point clouds as single session for 'multi_view_tls_registration_step_2' program");
        }
        if (step_1_done && step_2_done)
        {

            if (ImGui::Button("Save all point clouds to single las/laz file"))
            {
                const auto output_file_name = mandeye::fd::SaveFileDialog("Save las/laz file", mandeye::fd::LAS_LAZ_filter, ".laz");

                if (output_file_name.size() > 0)
                {
                    session.fill_session_from_worker_data(pipeline.worker_data, false, true, true, params.threshould_output_filter);
                    save_all_to_las(session, output_file_name, true, true);
                }
            }
        }
        if (!simple_gui)
        {
            ImGui::NewLine();
            ImGui::Checkbox("Use robust and accurate lidar odometry", &params.use_robust_and_accurate_lidar_odometry);

            if (params.use_robust_and_accurate_lidar_odometry)
            {
                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::InputDouble("Distance bucket [m]", &params.distance_bucket, 0.0, 0.0, "%.3f");
                ImGui::InputDouble("Polar angle [deg]", &params.polar_angle_deg, 0.0, 0.0, "%.3f");
                ImGui::InputDouble("Azimutal angle [deg]", &params.azimutal_angle_deg, 0.0, 0.0, "%.3f");
                ImGui::InputInt("Number of iterations", &params.robust_and_accurate_lidar_odometry_iterations);
                ImGui::PopItemWidth();
            }

            ImGui::NewLine();

            ImGui::Text("Rigid ICP using spherical coordinates");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("Distance bucket [m]##ricp", &params.distance_bucket_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputDouble("Polar angle [deg]##ricp", &params.polar_angle_deg_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputDouble("azimutal angle [deg]##ricp", &params.azimutal_angle_deg_rigid_sf, 0.0, 0.0, "%.3f");
            ImGui::InputInt("Number of iterations##ricp", &params.robust_and_accurate_lidar_odometry_rigid_sf_iterations);
            ImGui::InputDouble("Max distance [m]##ricp", &params.max_distance_lidar_rigid_sf, 0.0, 0.0, "%.2f");
            ImGui::PopItemWidth();

            ImGui::NewLine();

            ImGui::Text("Rigid spherical feature sigmas [m] / [deg]:");
            ImGui::PushItemWidth(ImGuiNumberWidth);
            ImGui::InputDouble("x##rgdsfs", &params.rgd_sf_sigma_x_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(xText);
            ImGui::SameLine();
            ImGui::InputDouble("om##rgdsfs", &params.rgd_sf_sigma_om_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(omText);

            ImGui::InputDouble("y##rgdsfs", &params.rgd_sf_sigma_y_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(yText);
            ImGui::SameLine();
            ImGui::InputDouble("fi##rgdsfs", &params.rgd_sf_sigma_fi_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(fiText);

            ImGui::InputDouble("z##rgdsfs", &params.rgd_sf_sigma_z_m, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(zText);
            ImGui::SameLine();
            ImGui::InputDouble("ka##rgdsfs", &params.rgd_sf_sigma_ka_deg, 0.0, 0.0, "%.3f");
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip(kaText);

            ImGui::PopItemWidth();

            ImGui::NewLine();

            if (ImGui::Button("Save trajectory to ASCII (x y z)"))
            {
                std::string output_file_name = "";
                output_file_name = mandeye::fd::SaveFileDialog("Save trajectory", {}, "");
                std::cout << "File to save: '" << output_file_name << "'" << std::endl;

                if (output_file_name.size() > 0)
                {
                    save_trajectory_to_ascii(pipeline.worker_data, output_file_name);
                }
            }

            ImGui::NewLine();
            ImGui::Text("Reference point clouds for initial alignment:");

            static std::vector<std::string> input_file_names;

            if (ImGui::Button("Load laz/las files"))
            {
                input_file_names = mandeye::fd::OpenFileDialog("Load laz/las files", mandeye::fd::LAS_LAZ_filter, true);
                if (input_file_names.size() > 0)
                {
                    show_reference_points = true;
                    load_reference_point_clouds(input_file_names, params, pipeline.state, console);
                }
            }

            if (!input_file_names.empty())
            {
                ImGui::Checkbox("Show points", &show_reference_points);
                ImGui::SameLine();
                ImGui::Checkbox("Show buckets", &show_reference_buckets);
                ImGui::SetNextItemWidth(ImGuiNumberWidth);
                ImGui::InputInt("Decimation###ref", &dec_reference_points);

                if (ImGui::Button("Filter reference buckets"))
                {
                    filter_reference_buckets(params, pipeline.state);
                }
            }

            if (pipeline.state.initial_points.size() > 0)
            {
                ImGui::NewLine();
                ImGui::Text("Manipulate initial transformation:");
                ImGui::Checkbox("Show gizmo", &initial_transformation_gizmo);
                if (initial_transformation_gizmo)
                {
                    m_gizmo[0] = (float)pipeline.state.m_g(0, 0);
                    m_gizmo[1] = (float)pipeline.state.m_g(1, 0);
                    m_gizmo[2] = (float)pipeline.state.m_g(2, 0);
                    m_gizmo[3] = (float)pipeline.state.m_g(3, 0);
                    m_gizmo[4] = (float)pipeline.state.m_g(0, 1);
                    m_gizmo[5] = (float)pipeline.state.m_g(1, 1);
                    m_gizmo[6] = (float)pipeline.state.m_g(2, 1);
                    m_gizmo[7] = (float)pipeline.state.m_g(3, 1);
                    m_gizmo[8] = (float)pipeline.state.m_g(0, 2);
                    m_gizmo[9] = (float)pipeline.state.m_g(1, 2);
                    m_gizmo[10] = (float)pipeline.state.m_g(2, 2);
                    m_gizmo[11] = (float)pipeline.state.m_g(3, 2);
                    m_gizmo[12] = (float)pipeline.state.m_g(0, 3);
                    m_gizmo[13] = (float)pipeline.state.m_g(1, 3);
                    m_gizmo[14] = (float)pipeline.state.m_g(2, 3);
                    m_gizmo[15] = (float)pipeline.state.m_g(3, 3);
                }
                else
                {
                    if (ImGui::Button("Align to reference"))
                    {
                        for (int i = 0; i < 30; i++)
                        {
                            align_to_reference(params.in_out_params_indoor, pipeline.state.initial_points, pipeline.state.m_g, pipeline.state.reference_buckets, console);
                        }
                    }
                }
            }

            if (pipeline.worker_data.size() > 0)
            {
                ImGui::NewLine();

                ImGui::Text("Scans selection:");

                ImGui::Text("index from: ");
                ImGui::SameLine();
                ImGui::PushItemWidth(ImGuiNumberWidth);
                ImGui::SliderInt("##fs", &index_begin, 0, index_end);
                ImGui::SameLine();
                ImGui::InputInt("##fi", &index_begin, 1, 5);
                if (index_begin < 0)
                    index_begin = 0;
                if (index_begin >= index_end)
                    index_begin = std::min(index_end, static_cast<int>(pipeline.worker_data.size() - 1));

                ImGui::SameLine();
                ImGui::Text(" to: ");
                ImGui::SameLine();

                int prev = index_end;
                ImGui::SliderInt("##ts", &index_end, index_begin, static_cast<int>(pipeline.worker_data.size() - 1));
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("max %zu", pipeline.worker_data.size() - 1);
                ImGui::SameLine();
                ImGui::InputInt("##ti", &index_end, 1, 5);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("max %zu", pipeline.worker_data.size() - 1);
                if (index_end < index_begin)
                    index_end = index_begin;
                if (index_end >= pipeline.worker_data.size() - 1)
                    index_end = pipeline.worker_data.size() - 1;
                if (prev != index_end)
                {
                    stretch_gizmo_m = pipeline.worker_data[index_end].intermediate_trajectory[0];
                }

                ImGui::PopItemWidth();

                ImGui::Text("Selection: ");
                if (ImGui::Button("Select all"))
                {
                    for (size_t k = 0; k < pipeline.worker_data.size(); k++)
                    {
                        pipeline.worker_data[k].show = true;
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("Unselect"))
                {
                    for (size_t k = 0; k < pipeline.worker_data.size(); k++)
                    {
                        pipeline.worker_data[k].show = false;
                    }
                }

                ImGui::Text("Select scans from range:");
                ImGui::SameLine();
                if (ImGui::Button("<from, to>"))
                {
                    for (size_t k = 0; k < pipeline.worker_data.size(); k++)
                        pipeline.worker_data[k].show = (k >= index_begin && k <= index_end);
                }

                if (ImGui::Button("<from - 1, to - 1>"))
                {
                    if (index_begin > 1 && index_end > 1)
                    {
                        index_begin--;
                        index_end--;

                        for (size_t k = 0; k < pipeline.worker_data.size(); k++)
                            pipeline.worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("<from + 1, to + 1>"))
                {
                    if (index_begin + 1 < pipeline.worker_data.size() && index_end + 1 < pipeline.worker_data.size())
                    {
                        index_begin++;
                        index_end++;

                        for (size_t k = 0; k < pipeline.worker_data.size(); k++)
                            pipeline.worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }

                if (ImGui::Button("from - 10, to - 10>"))
                {
                    if (index_begin > 10 && index_end > 10)
                    {
                        index_begin -= 10;
                        index_end -= 10;

                        for (size_t k = 0; k < pipeline.worker_data.size(); k++)
                            pipeline.worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }
                ImGui::SameLine();
                if (ImGui::Button("<from + 10, to + 10>"))
                {
                    if (index_begin + 10 < pipeline.worker_data.size() && index_end + 10 < pipeline.worker_data.size())
                    {
                        index_begin += 10;
                        index_end += 10;

                        for (size_t k = 0; k < pipeline.worker_data.size(); k++)
                            pipeline.worker_data[k].show = (k >= index_begin && k <= index_end);
                    }
                }

                if (ImGui::Button("Export selected scans"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save las/laz file", mandeye::fd::LAS_LAZ_filter, ".laz");
                    Eigen::Affine3d pose;
                    if (output_file_name.size() > 0)
                    {
                        session.fill_session_from_worker_data(pipeline.worker_data, true, false, false, params.threshould_output_filter);
                        save_all_to_las(session, output_file_name, false, true);
                    }
                    // TODO: give value to pose even if output_file_name is wrong

                    std::cout << "----------------------------------------" << std::endl;
                    std::cout << "please add following lines to RESSO file:" << std::endl;
                    std::cout << fs::path(output_file_name).filename() << "(!!!please remove brackets!!!)" << std::endl;
                    std::cout << pose.matrix() << std::endl;

                    std::cout << "example RESSO file" << std::endl;
                    std::cout << ".................................................." << std::endl;
                    std::cout << "3" << std::endl
                              << "scan_0.laz" << std::endl
                              << "0.999775 0.000552479 -0.0212158 -0.0251188" << std::endl
                              << "0.000834612 0.997864 0.0653156 -0.0381429" << std::endl
                              << "0.0212066 - 0.0653186 0.997639 -0.000757752" << "0 0 0 1" << std::endl
                              << "scan_1.laz" << std::endl
                              << "0.999783 0.00178963 -0.0207603 -0.0309683" << std::endl
                              << "-0.000467341 0.99798 0.0635239 -0.0517512" << std::endl
                              << "0.0208321 -0.0635004 0.997764 0.00331449" << std::endl
                              << "0 0 0 1" << std::endl
                              << "scan_2.laz" << std::endl
                              << "0.999783 0.00163449 -0.0207736 -0.0309985" << std::endl
                              << "-0.000312224 0.997982 0.0634957 -0.0506113" << std::endl
                              << "0.0208355 -0.0634754 0.997766 0.0028499" << std::endl
                              << "0 0 0 1" << std::endl;
                    std::cout << "................................................." << std::endl;
                }

                ImGui::SameLine();

                if (ImGui::Button("Save RESSO file"))
                {
                    auto output_file_name = mandeye::fd::SaveFileDialog("Save RESSO file", {}, "");
                    std::cout << "RESSO file to save: '" << output_file_name << "'" << std::endl;

                    if (output_file_name.size() > 0)
                    {
                        session.point_clouds_container.save_poses(fs::path(output_file_name).string(), false);
                    }
                }

                if (index_end > index_begin)
                {
                    ImGui::Separator();
                    ImGui::Checkbox("Gizmo_stretch_interval", &gizmo_stretch_interval);
                }

                if (gizmo_stretch_interval)
                {
                    if (index_end < pipeline.worker_data.size())
                    {
                        if (pipeline.worker_data[index_end].intermediate_trajectory.size() > 0)
                        {
                            m_gizmo[0] = (float)stretch_gizmo_m(0, 0);
                            m_gizmo[1] = (float)stretch_gizmo_m(1, 0);
                            m_gizmo[2] = (float)stretch_gizmo_m(2, 0);
                            m_gizmo[3] = (float)stretch_gizmo_m(3, 0);
                            m_gizmo[4] = (float)stretch_gizmo_m(0, 1);
                            m_gizmo[5] = (float)stretch_gizmo_m(1, 1);
                            m_gizmo[6] = (float)stretch_gizmo_m(2, 1);
                            m_gizmo[7] = (float)stretch_gizmo_m(3, 1);
                            m_gizmo[8] = (float)stretch_gizmo_m(0, 2);
                            m_gizmo[9] = (float)stretch_gizmo_m(1, 2);
                            m_gizmo[10] = (float)stretch_gizmo_m(2, 2);
                            m_gizmo[11] = (float)stretch_gizmo_m(3, 2);
                            m_gizmo[12] = (float)stretch_gizmo_m(0, 3);
                            m_gizmo[13] = (float)stretch_gizmo_m(1, 3);
                            m_gizmo[14] = (float)stretch_gizmo_m(2, 3);
                            m_gizmo[15] = (float)stretch_gizmo_m(3, 3);
                        }
                    }

                    if (ImGui::Button("Accept Gizmo (only translation)"))
                    {
                        if (index_end < pipeline.worker_data.size())
                        {
                            if (pipeline.worker_data[index_end].intermediate_trajectory.size() > 0)
                            {
                                Eigen::Affine3d current_gizmo = Eigen::Affine3d::Identity();
                                current_gizmo(0, 0) = m_gizmo[0];
                                current_gizmo(1, 0) = m_gizmo[1];
                                current_gizmo(2, 0) = m_gizmo[2];
                                current_gizmo(3, 0) = m_gizmo[3];
                                current_gizmo(0, 1) = m_gizmo[4];
                                current_gizmo(1, 1) = m_gizmo[5];
                                current_gizmo(2, 1) = m_gizmo[6];
                                current_gizmo(3, 1) = m_gizmo[7];
                                current_gizmo(0, 2) = m_gizmo[8];
                                current_gizmo(1, 2) = m_gizmo[9];
                                current_gizmo(2, 2) = m_gizmo[10];
                                current_gizmo(3, 2) = m_gizmo[11];
                                current_gizmo(0, 3) = m_gizmo[12];
                                current_gizmo(1, 3) = m_gizmo[13];
                                current_gizmo(2, 3) = m_gizmo[14];
                                current_gizmo(3, 3) = m_gizmo[15];

                                auto first_pose = pipeline.worker_data[index_begin].intermediate_trajectory[0];

                                Eigen::Vector3d translation = current_gizmo.translation() - first_pose.translation();

                                float number_all_nodes_inside_interval = 0;
                                for (int i = index_begin; i < index_end; i++)
                                {
                                    number_all_nodes_inside_interval += (float)pipeline.worker_data[i].intermediate_trajectory.size();
                                }

                                std::vector<std::vector<Eigen::Affine3d>> all_poses;
                                for (size_t i = 0; i < pipeline.worker_data.size(); i++)
                                {
                                    std::vector<Eigen::Affine3d> poses;
                                    for (size_t j = 0; j < pipeline.worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        poses.push_back(pipeline.worker_data[i].intermediate_trajectory[j]);
                                    }
                                    all_poses.push_back(poses);
                                }

                                float counter = 0;

                                Eigen::Affine3d last_m = Eigen::Affine3d::Identity();

                                for (int i = index_begin; i < index_end; i++)
                                {
                                    for (size_t j = 0; j < pipeline.worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(pipeline.worker_data[i].intermediate_trajectory[j]);

                                        pose.px = first_pose.translation().x() + translation.x() * (counter / number_all_nodes_inside_interval);
                                        pose.py = first_pose.translation().y() + translation.y() * (counter / number_all_nodes_inside_interval);
                                        pose.pz = first_pose.translation().z() + translation.z() * (counter / number_all_nodes_inside_interval);

                                        counter += 1.0f;

                                        pipeline.worker_data[i].intermediate_trajectory[j] = affine_matrix_from_pose_tait_bryan(pose);

                                        last_m = pipeline.worker_data[i].intermediate_trajectory[j];
                                    }
                                }

                                for (size_t i = index_end; i < pipeline.worker_data.size(); i++)
                                {
                                    for (size_t j = 0; j < pipeline.worker_data[i].intermediate_trajectory.size(); j++)
                                    {
                                        Eigen::Affine3d m_update = Eigen::Affine3d::Identity();

                                        if (j == 0)
                                        {
                                            m_update = all_poses[i - 1][all_poses[i - 1].size() - 1].inverse() * all_poses[i][0];
                                        }
                                        else
                                        {
                                            m_update = all_poses[i][j - 1].inverse() * all_poses[i][j];
                                        }
                                        last_m = last_m * m_update;
                                        pipeline.worker_data[i].intermediate_trajectory[j] = last_m;
                                    }
                                }
                            }
                        }
                    }
                    ImGui::Separator();
                }

                ImGui::NewLine();
                ImGui::Text("Show/hide individual scans:");

                int n_items = (int)pipeline.worker_data.size();
                int digits = (n_items > 0) ? (int)std::log10(n_items - 1) + 1 : 1;

                for (int i = 0; i < n_items; i++)
                {
                    std::stringstream ss;
                    ss << "[" << std::setw(digits) << std::setfill('0') << i << "]";

                    ImGui::Checkbox(ss.str().c_str(), &pipeline.worker_data[i].show);

                    // Arrange checkboxes in rows of 5
                    if ((i + 1) % 5 != 0)
                        ImGui::SameLine();
                }
            }
        }
    }

    ImGui::End();
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
    is_settings_gui = false;
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

                save_results(false, elapsed_seconds.count());

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

// the current run's points, trajectory and reference data, and the interval markers
void draw_run_data()
{
    if (show_initial_points)
    {
        glColor3d(0.0, 1.0, 0.0);
        glPointSize(point_size);
        glBegin(GL_POINTS);
        for (const auto &p : pipeline.state.initial_points)
        {
            auto pp = pipeline.state.m_g * p.point;
            glVertex3d(pp.x(), pp.y(), pp.z());
        }
        glEnd();
    }

    if (show_reference_points)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < pipeline.state.reference_points.size(); i += dec_reference_points)
            glVertex3f(pipeline.state.reference_points[i].point.x(), pipeline.state.reference_points[i].point.y(), pipeline.state.reference_points[i].point.z());
        glEnd();
    }

    if (show_trajectory_as_axes)
    {
        glBegin(GL_LINES);
        for (const auto &wd : pipeline.worker_data)
        {
            for (const auto &it : wd.intermediate_trajectory)
            {
                glColor3f(1, 0, 0);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 0) * 0.1, it(1, 3) + it(1, 0) * 0.1, it(2, 3) + it(2, 0) * 0.1);

                glColor3f(0, 1, 0);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 1) * 0.1, it(1, 3) + it(1, 1) * 0.1, it(2, 3) + it(2, 1) * 0.1);

                glColor3f(0, 0, 1);
                glVertex3f(it(0, 3), it(1, 3), it(2, 3));
                glVertex3f(it(0, 3) + it(0, 2) * 0.1, it(1, 3) + it(1, 2) * 0.1, it(2, 3) + it(2, 2) * 0.1);
            }
        }
        glEnd();
    }

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

    if (show_reference_buckets)
    {
        glColor3f(1, 0, 0);
        glBegin(GL_POINTS);
        for (const auto &b : pipeline.state.reference_buckets)
            glVertex3f(b.second.mean.x(), b.second.mean.y(), b.second.mean.z());
        glEnd();
    }

    if (pipeline.worker_data.size() > 0)
    {
        if (index_begin < pipeline.worker_data.size())
        {
            glColor3f(1, 1, 0);
            glBegin(GL_LINES);
            if (pipeline.worker_data[index_begin].intermediate_trajectory.size() > 0)
            {
                auto p = pipeline.worker_data[index_begin].intermediate_trajectory[0].translation();
                glVertex3f(p.x() - 1, p.y(), p.z());
                glVertex3f(p.x() + 1, p.y(), p.z());

                glVertex3f(p.x(), p.y() - 1, p.z());
                glVertex3f(p.x(), p.y() + 1, p.z());

                glVertex3f(p.x(), p.y(), p.z() - 1);
                glVertex3f(p.x(), p.y(), p.z() + 1);
            }
            glEnd();
        }

        if (index_end < pipeline.worker_data.size())
        {
            glColor3f(0, 1, 1);
            glBegin(GL_LINES);
            if (pipeline.worker_data[index_end].intermediate_trajectory.size() > 0)
            {
                auto p = pipeline.worker_data[index_end].intermediate_trajectory[0].translation();
                glVertex3f(p.x() - 1, p.y(), p.z());
                glVertex3f(p.x() + 1, p.y(), p.z());

                glVertex3f(p.x(), p.y() - 1, p.z());
                glVertex3f(p.x(), p.y() + 1, p.z());

                glVertex3f(p.x(), p.y(), p.z() - 1);
                glVertex3f(p.x(), p.y(), p.z() + 1);
            }
            glEnd();
        }
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
                ImGui::BeginDisabled(!(session.point_clouds_container.point_clouds.size() > 0));
                {
                    auto tmp = point_size;
                    ImGui::SetNextItemWidth(ImGuiNumberWidth);
                    ImGui::InputInt("Points size", &point_size);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("keyboard 1-9 keys");
                    if (point_size < 1)
                        point_size = 1;
                    else if (point_size > 10)
                        point_size = 10;

                    if (tmp != point_size)
                        for (auto& point_cloud : session.point_clouds_container.point_clouds)
                            point_cloud.point_size = point_size;

                    ImGui::Separator();
                }
                ImGui::EndDisabled();

                ImGui::MenuItem("Show initial points", nullptr, &show_initial_points);
                ImGui::MenuItem("Show trajectory", nullptr, &show_trajectory);
                ImGui::MenuItem("Show trajectory as axes", nullptr, &show_trajectory_as_axes);
                ImGui::MenuItem("Show compass/ruler", "key C", &compass_ruler);

                ImGui::MenuItem("Lock Z", "Shift + Z", &lock_z, !is_ortho);


                ImGui::Separator();

                ImGui::MenuItem("Settings", nullptr, &is_settings_gui);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Show power user settings window with more parameters");

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

    if (initial_transformation_gizmo)
    {
        ImGuiIO &io = ImGui::GetIO();
        // ImGuizmo -----------------------------------------------
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        GLfloat projection[16];
        glGetFloatv(GL_PROJECTION_MATRIX, projection);

        GLfloat modelview[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

        ImGuizmo::Manipulate(&modelview[0], &projection[0], ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y, ImGuizmo::WORLD, m_gizmo, NULL);

        pipeline.state.m_g(0, 0) = m_gizmo[0];
        pipeline.state.m_g(1, 0) = m_gizmo[1];
        pipeline.state.m_g(2, 0) = m_gizmo[2];
        pipeline.state.m_g(3, 0) = m_gizmo[3];
        pipeline.state.m_g(0, 1) = m_gizmo[4];
        pipeline.state.m_g(1, 1) = m_gizmo[5];
        pipeline.state.m_g(2, 1) = m_gizmo[6];
        pipeline.state.m_g(3, 1) = m_gizmo[7];
        pipeline.state.m_g(0, 2) = m_gizmo[8];
        pipeline.state.m_g(1, 2) = m_gizmo[9];
        pipeline.state.m_g(2, 2) = m_gizmo[10];
        pipeline.state.m_g(3, 2) = m_gizmo[11];
        pipeline.state.m_g(0, 3) = m_gizmo[12];
        pipeline.state.m_g(1, 3) = m_gizmo[13];
        pipeline.state.m_g(2, 3) = m_gizmo[14];
        pipeline.state.m_g(3, 3) = m_gizmo[15];
    }

    if (gizmo_stretch_interval)
    {
        ImGuiIO &io = ImGui::GetIO();
        // ImGuizmo -----------------------------------------------
        ImGuizmo::BeginFrame();
        ImGuizmo::Enable(true);
        ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);

        GLfloat projection[16];
        glGetFloatv(GL_PROJECTION_MATRIX, projection);

        GLfloat modelview[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, modelview);

        ImGuizmo::Manipulate(&modelview[0], &projection[0], ImGuizmo::TRANSLATE | ImGuizmo::ROTATE_Z | ImGuizmo::ROTATE_X | ImGuizmo::ROTATE_Y, ImGuizmo::WORLD, m_gizmo, NULL);

        stretch_gizmo_m(0, 0) = m_gizmo[0];
        stretch_gizmo_m(1, 0) = m_gizmo[1];
        stretch_gizmo_m(2, 0) = m_gizmo[2];
        stretch_gizmo_m(3, 0) = m_gizmo[3];
        stretch_gizmo_m(0, 1) = m_gizmo[4];
        stretch_gizmo_m(1, 1) = m_gizmo[5];
        stretch_gizmo_m(2, 1) = m_gizmo[6];
        stretch_gizmo_m(3, 1) = m_gizmo[7];
        stretch_gizmo_m(0, 2) = m_gizmo[8];
        stretch_gizmo_m(1, 2) = m_gizmo[9];
        stretch_gizmo_m(2, 2) = m_gizmo[10];
        stretch_gizmo_m(3, 2) = m_gizmo[11];
        stretch_gizmo_m(0, 3) = m_gizmo[12];
        stretch_gizmo_m(1, 3) = m_gizmo[13];
        stretch_gizmo_m(2, 3) = m_gizmo[14];
        stretch_gizmo_m(3, 3) = m_gizmo[15];
    }

    if (is_settings_gui)
    {
        settings_gui();
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