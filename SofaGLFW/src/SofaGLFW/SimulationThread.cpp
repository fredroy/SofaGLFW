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
#include <SofaGLFW/SimulationThread.h>

#include <sofa/helper/logging/Messaging.h>

#include <exception>

#if defined(__APPLE__) || defined(__linux__)
#include <pthread.h>
#endif

namespace sofaglfw
{

namespace
{
    void nameCurrentThread(const char* name)
    {
#if defined(__APPLE__)
        pthread_setname_np(name);
#elif defined(__linux__)
        pthread_setname_np(pthread_self(), name);
#else
        SOFA_UNUSED(name);
#endif
    }
}

SimulationThread::SimulationThread(std::function<bool()> canStep, std::function<void()> step)
    : m_canStep(std::move(canStep))
    , m_step(std::move(step))
{
}

SimulationThread::~SimulationThread()
{
    stop();
}

void SimulationThread::start()
{
    if (m_thread.joinable())
        return;
    {
        std::lock_guard lock(m_mutex);
        m_owner = Owner::None;
        m_guiWaiting = false;
        m_guiYieldsForStep = false;
        m_stopRequested = false;
        m_failed = false;
        m_canStepNow = false;
        m_running = true;
    }
    m_thread = std::thread([this] { run(); });
}

void SimulationThread::stop()
{
    if (!m_thread.joinable())
        return;
    {
        std::lock_guard lock(m_mutex);
        m_stopRequested = true;
    }
    m_simulationCondition.notify_all();
    m_thread.join();
}

void SimulationThread::run()
{
    nameCurrentThread("SOFA simulation");

    std::unique_lock lock(m_mutex);
    while (true)
    {
        m_simulationCondition.wait(lock, [this]
        {
            if (m_stopRequested)
                return true;
            if (m_owner != Owner::None)
                return false; // the GUI holds the scene
            // canStep reads the scene: nobody owns it, and the GUI cannot take it
            // without the mutex.
            m_canStepNow = !m_failed && m_canStep();
            if (!m_canStepNow && m_guiYieldsForStep)
            {
                // No step to wait for: the GUI takes the scene.
                m_guiYieldsForStep = false;
                m_guiCondition.notify_all();
            }
            return m_canStepNow && (!m_guiWaiting || m_guiYieldsForStep);
        });
        if (m_stopRequested)
            break;

        m_owner = Owner::Simulation;
        lock.unlock();

        bool failed = false;
        try
        {
            m_step();
        }
        catch (const std::exception& e)
        {
            msg_error("SimulationThread") << "The simulation step failed: " << e.what()
                                          << ". The simulation thread does no more steps.";
            failed = true;
        }
        catch (...)
        {
            msg_error("SimulationThread") << "The simulation step failed with an unknown exception. "
                                             "The simulation thread does no more steps.";
            failed = true;
        }

        lock.lock();
        m_owner = Owner::None;
        m_failed = m_failed || failed;
        ++m_nbSteps;
        m_guiYieldsForStep = false;
        m_guiCondition.notify_all(); // the GUI may be waiting for the scene
    }

    m_running = false;
    m_guiYieldsForStep = false;
    m_guiCondition.notify_all();
}

SimulationThread::SceneLock SimulationThread::lockScene()
{
    return acquire(nullptr);
}

SimulationThread::SceneLock SimulationThread::lockScene(Clock::time_point deadline)
{
    return acquire(&deadline);
}

SimulationThread::SceneLock SimulationThread::acquire(const Clock::time_point* deadline)
{
    std::unique_lock lock(m_mutex);
    m_guiWaiting = true;

    // No step since the last release while the simulation can step: wait for one (the
    // step in progress, if any).
    if (m_running && m_canStepNow && m_nbSteps == m_nbStepsAtRelease && !m_guiYieldsForStep)
    {
        m_guiYieldsForStep = true;
        m_simulationCondition.notify_all();
    }

    const auto sceneAvailable = [this] { return m_owner == Owner::None && !m_guiYieldsForStep; };
    if (deadline)
    {
        if (!m_guiCondition.wait_until(lock, *deadline, sceneAvailable))
            return SceneLock(this, false); // m_guiWaiting stays: the GUI comes back
    }
    else
    {
        m_guiCondition.wait(lock, sceneAvailable);
    }

    m_owner = Owner::GUI;
    m_guiWaiting = false;
    return SceneLock(this, true);
}

void SimulationThread::release()
{
    {
        std::lock_guard lock(m_mutex);
        m_owner = Owner::None;
        m_nbStepsAtRelease = m_nbSteps;
    }
    m_simulationCondition.notify_all();
}

SimulationThread::SceneLock::SceneLock(SimulationThread* owner, bool ownsScene)
    : m_owner(owner)
    , m_ownsScene(ownsScene)
{
}

SimulationThread::SceneLock::~SceneLock()
{
    unlock();
}

SimulationThread::SceneLock::SceneLock(SceneLock&& other) noexcept
    : m_owner(other.m_owner)
    , m_ownsScene(other.m_ownsScene)
{
    other.m_owner = nullptr;
}

SimulationThread::SceneLock& SimulationThread::SceneLock::operator=(SceneLock&& other) noexcept
{
    if (this != &other)
    {
        unlock();
        m_owner = other.m_owner;
        m_ownsScene = other.m_ownsScene;
        other.m_owner = nullptr;
    }
    return *this;
}

void SimulationThread::SceneLock::unlock()
{
    // A lock that got no scene leaves the GUI's request: it comes back for the scene.
    if (m_owner && m_ownsScene)
        m_owner->release();
    m_owner = nullptr;
}

} // namespace sofaglfw
