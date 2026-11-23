/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once
#include <SofaGLFW/config.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

namespace sofaglfw
{

/// Runs the simulation steps on a thread of its own, so that they go on while the GUI
/// thread presents its frames (and waits for the display).
///
/// The scene has a single owner at a time: the simulation thread while it steps, the
/// GUI thread while it holds a SceneLock, for everything that reads or writes the scene
/// (events, scene draw, GUI panels, loading). Nothing else runs in parallel with a step:
/// in particular, the graphics API work done without the scene (submitting and
/// presenting the frame) must not touch it.
///
/// The simulation gets at least one step between two scene locks of the GUI (when it
/// can step): a GUI drawing frames back to back, which nothing makes wait for the
/// display, would otherwise hold the scene almost all the time. At worst, it alternates
/// steps and frames, as when the GUI thread does the steps itself.
///
/// A step may be long. The GUI can then give up waiting at a deadline and draw a frame
/// without the scene: its request stays, so the thread starts no new step after the
/// current one until the GUI comes back for the scene.
class SOFAGLFW_API SimulationThread
{
public:
    using Clock = std::chrono::steady_clock;

    /// Both run on the simulation thread, which owns the scene meanwhile.
    /// @param canStep whether a step is to be done now (the simulation is animated...);
    ///        re-evaluated after each step and each time the GUI releases the scene.
    /// @param step one simulation step.
    SimulationThread(std::function<bool()> canStep, std::function<void()> step);
    /// Stops the thread. Must not be called while the GUI holds the scene.
    ~SimulationThread();

    SimulationThread(const SimulationThread&) = delete;
    SimulationThread& operator=(const SimulationThread&) = delete;

    void start();
    /// Waits for the step in progress and joins the thread. Must not be called while
    /// the GUI holds the scene.
    void stop();
    bool isRunning() const { return m_thread.joinable(); }

    /// Access to the scene for the GUI thread, released on destruction. Without a
    /// simulation thread (default-constructed lock), the scene is always the GUI's.
    class SOFAGLFW_API SceneLock
    {
    public:
        SceneLock() = default;
        ~SceneLock();
        SceneLock(SceneLock&& other) noexcept;
        SceneLock& operator=(SceneLock&& other) noexcept;
        SceneLock(const SceneLock&) = delete;
        SceneLock& operator=(const SceneLock&) = delete;

        /// Whether the GUI thread may read and write the scene.
        bool ownsScene() const { return m_ownsScene; }
        explicit operator bool() const { return m_ownsScene; }

        void unlock();

    private:
        friend class SimulationThread;
        SceneLock(SimulationThread* owner, bool ownsScene);

        SimulationThread* m_owner{nullptr};
        bool m_ownsScene{true};
    };

    /// Waits as long as needed for the scene. Not reentrant: the GUI thread must not
    /// hold two locks.
    [[nodiscard]] SceneLock lockScene();
    /// Waits for the scene until the deadline at most: the lock owns no scene if it
    /// was not available by then.
    [[nodiscard]] SceneLock lockScene(Clock::time_point deadline);

private:
    enum class Owner { None, Simulation, GUI };

    void run();
    SceneLock acquire(const Clock::time_point* deadline);
    void release();

    std::function<bool()> m_canStep;
    std::function<void()> m_step;

    /// Guards everything below (except m_thread); held briefly, never during a step or
    /// while the GUI holds the scene.
    std::mutex m_mutex;
    /// The simulation thread waits on it: scene released, stop requested.
    std::condition_variable m_simulationCondition;
    /// The GUI thread waits on it for the scene.
    std::condition_variable m_guiCondition;
    Owner m_owner{Owner::None};
    /// The GUI waits for the scene, or gave up at a deadline and comes back: the
    /// simulation thread starts no step, unless the GUI yields for one.
    bool m_guiWaiting{false};
    /// The GUI lets one step run before it takes the scene.
    bool m_guiYieldsForStep{false};
    bool m_running{false}; ///< the thread runs its loop
    bool m_stopRequested{false};
    bool m_failed{false}; ///< a step threw: no more steps
    /// Result of the last evaluation of canStep.
    bool m_canStepNow{false};
    std::uint64_t m_nbSteps{0};
    std::uint64_t m_nbStepsAtRelease{0}; ///< when the GUI last released the scene
    std::thread m_thread;
};

} // namespace sofaglfw
