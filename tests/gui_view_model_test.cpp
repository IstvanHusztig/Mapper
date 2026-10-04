#include <doctest/doctest.h>

#include "gui_view_model.h"

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

// The GUI's view-model without a display: the settings and their source, starting a run, and the progress window's
// Pause/Resume. A recording notifier stands in for the view's message boxes.

namespace fs = std::filesystem;

namespace
{
    const fs::path repo_dir = LO_REPO_DIR;

    struct RecordingNotifier : RunNotifier
    {
        std::vector<std::string> load_failures;
        int saved = 0;
        void load_failed(const std::string &folder) override { load_failures.push_back(folder); }
        void run_saved(const std::string &, double, double) override { saved++; }
    };

    // an empty folder of its own, removed again at the end of the test
    struct TemporaryFolder
    {
        fs::path path = fs::temp_directory_path() / "lo_gui_view_model_test_empty";
        TemporaryFolder()
        {
            fs::remove_all(path);
            fs::create_directories(path);
        }
        ~TemporaryFolder() { fs::remove_all(path); }
    };
}

TEST_CASE("gui view-model: default settings, and their source")
{
    RecordingNotifier notifier;
    GuiViewModel model(notifier);
    CHECK(model.active_parameters() == "built-in defaults");

    SUBCASE("a missing default file keeps the built-in defaults")
    {
        model.load_default_parameters(repo_dir / "tests" / "data" / "no_such_settings.toml");
        CHECK(model.active_parameters() == "built-in defaults");
        CHECK_FALSE(model.config().use_loop_closure);
    }
    SUBCASE("a default file is loaded and named")
    {
        const fs::path file = repo_dir / "tests" / "data" / "lidar_vertical.golden.toml";
        model.load_default_parameters(file);
        CHECK(model.active_parameters() == file.string());
        CHECK(model.config().use_loop_closure);
        CHECK(model.config().threshold_nr_poses == 360);
    }
}

TEST_CASE("gui view-model: Load parameters")
{
    RecordingNotifier notifier;
    GuiViewModel model(notifier);
    const std::string file = (repo_dir / "tests" / "data" / "lidar_vertical.golden.toml").string();
    REQUIRE(model.load_parameters(file));
    CHECK(model.active_parameters() == file);
    CHECK(model.config().use_consistency);

    // an unreadable file changes nothing
    CHECK_FALSE(model.load_parameters((repo_dir / "tests" / "data" / "no_such_settings.toml").string()));
    CHECK(model.active_parameters() == file);
    CHECK(model.config().use_consistency);
}

TEST_CASE("gui view-model: a run on a folder that doesn't exist doesn't start")
{
    RecordingNotifier notifier;
    GuiViewModel model(notifier);
    CHECK_FALSE(model.start_run((fs::temp_directory_path() / "lo_gui_view_model_test_no_such_folder").string()));
    CHECK_FALSE(model.running());
    model.wait_for_run();
    CHECK(notifier.load_failures.empty());
    CHECK(model.run_parameters().empty());
}

TEST_CASE("gui view-model: a run on an empty folder fails to load")
{
    TemporaryFolder folder;
    RecordingNotifier notifier;
    GuiViewModel model(notifier);
    const std::string file = (repo_dir / "tests" / "data" / "lidar_vertical.golden.toml").string();
    REQUIRE(model.load_parameters(file));

    REQUIRE(model.start_run(folder.path.string()));
    model.wait_for_run();
    CHECK_FALSE(model.running());
    REQUIRE(notifier.load_failures.size() == 1);
    CHECK(notifier.load_failures[0] == folder.path.string());
    CHECK(notifier.saved == 0);
    CHECK(model.run_parameters() == file); // the source when the run started
    CHECK(model.working_directory().empty()); // no file to take it from
    CHECK_FALSE(fs::exists(folder.path / "cache"));
}

TEST_CASE("gui progress: pause holds the run at its next checkpoint until resume; reset starts afresh")
{
    struct SilentReporter : ProgressReporter
    {
        void stage_started(RunStage) override {}
        void progress(std::size_t, std::size_t) override {}
        void checkpoint() override {}
    } console;
    GuiProgress progress(console);

    progress.stage_started(RunStage::Odometry);
    progress.progress(3, 10);
    CHECK(progress.windows_done() == 3);
    CHECK(progress.fraction() == doctest::Approx(0.3));

    progress.set_paused(true);
    std::atomic<bool> passed{false};
    std::thread run([&] {
        progress.checkpoint();
        passed = true;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK_FALSE(passed.load());
    progress.set_paused(false);
    run.join();
    CHECK(passed.load());

    progress.set_paused(true);
    progress.reset();
    CHECK_FALSE(progress.paused());
    CHECK(progress.windows_done() == 0);
    CHECK(progress.stage() == RunStage::Loading);
    CHECK(progress.fraction() == 0.0f);
}
