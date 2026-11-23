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

#include <atomic>
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
/// The scene has a single owner at a time. The simulation thread holds it while it
/// steps; the GUI thread takes it with lockScene() for everything that reads or writes
/// the scene (events, scene draw, GUI panels, loading), which waits for the step in
/// progress and holds the next ones until the lock is released. Nothing else runs in
/// parallel with a step: in particular, the graphics API work done outside the lock
/// (submitting and presenting the frame) must not touch the scene.
///
/// The simulation gets at least one step between two locks of the GUI (when it can
/// step): a GUI drawing frames back to back, which nothing makes wait for the display,
/// would otherwise hold the scene almost all the time. At worst, it alternates steps and
/// frames, as when the GUI thread does the steps itself.
class SOFAGLFW_API SimulationThread
{
public:
    /// Both run on the simulation thread, with the scene lock held.
    /// @param canStep whether a step is to be done now (the simulation is animated...);
    ///        re-evaluated after each step and each time the GUI releases the scene.
    /// @param step one simulation step.
    SimulationThread(std::function<bool()> canStep, std::function<void()> step);
    /// Stops the thread. Must not be called with the scene lock held.
    ~SimulationThread();

    SimulationThread(const SimulationThread&) = delete;
    SimulationThread& operator=(const SimulationThread&) = delete;

    void start();
    /// Waits for the step in progress and joins the thread. Must not be called with
    /// the scene lock held (the thread needs it to finish its step).
    void stop();
    bool isRunning() const { return m_thread.joinable(); }

    /// Exclusive access to the scene for the GUI thread, released on destruction. A
    /// default-constructed lock (no simulation thread) holds nothing.
    class SOFAGLFW_API SceneLock
    {
    public:
        SceneLock() = default;
        explicit SceneLock(SimulationThread* owner);
        ~SceneLock();
        SceneLock(SceneLock&& other) noexcept;
        SceneLock& operator=(SceneLock&& other) noexcept;
        SceneLock(const SceneLock&) = delete;
        SceneLock& operator=(const SceneLock&) = delete;

        void unlock();

    private:
        SimulationThread* m_owner{nullptr};
    };

    /// Not reentrant: the GUI thread must not take it twice.
    [[nodiscard]] SceneLock lockScene() { return SceneLock(this); }

private:
    void run();

    std::function<bool()> m_canStep;
    std::function<void()> m_step;

    /// The scene. Everything below is accessed with it, unless stated otherwise.
    std::mutex m_mutex;
    /// The simulation thread waits on it: GUI released the scene, stop requested.
    std::condition_variable m_simulationCondition;
    /// The GUI thread waits on it for the step it yields the scene for.
    std::condition_variable m_guiCondition;
    /// GUI threads waiting for (or holding) the scene: the simulation thread does not
    /// start a step while it is not zero, unless the GUI yields for one. Incremented
    /// without m_mutex (the simulation thread holds it during its steps); decremented
    /// with it, so that the simulation thread cannot miss the wake-up that follows.
    std::atomic<int> m_guiRequests{0};
    /// The GUI waits for one step before taking the scene.
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
