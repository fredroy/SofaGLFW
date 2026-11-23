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

void SimulationThread::notify()
{
    // Taking the mutex orders the change of the condition before the wake-up.
    {
        std::lock_guard lock(m_mutex);
    }
    m_simulationCondition.notify_all();
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
            m_canStepNow = !m_failed && m_canStep();
            if (!m_canStepNow && m_guiYieldsForStep)
            {
                // No step to wait for: the GUI takes the scene back.
                m_guiYieldsForStep = false;
                m_guiCondition.notify_all();
            }
            return m_canStepNow && (m_guiRequests.load() == 0 || m_guiYieldsForStep);
        });
        if (m_stopRequested)
            break;

        try
        {
            m_step();
        }
        catch (const std::exception& e)
        {
            msg_error("SimulationThread") << "The simulation step failed: " << e.what()
                                          << ". The simulation thread does no more steps.";
            m_failed = true;
        }
        catch (...)
        {
            msg_error("SimulationThread") << "The simulation step failed with an unknown exception. "
                                             "The simulation thread does no more steps.";
            m_failed = true;
        }
        ++m_nbSteps;

        if (m_guiYieldsForStep)
        {
            m_guiYieldsForStep = false;
            m_guiCondition.notify_all();
        }
    }

    m_running = false;
    m_guiYieldsForStep = false;
    m_guiCondition.notify_all();
}

SimulationThread::SceneLock::SceneLock(SimulationThread* owner)
    : m_owner(owner)
{
    if (!m_owner)
        return;

    // Announced before waiting: the simulation thread finishes its step and lets go.
    m_owner->m_guiRequests.fetch_add(1);
    std::unique_lock lock(m_owner->m_mutex);

    // No step since the last release while the simulation could step: yield for one.
    if (m_owner->m_running && m_owner->m_canStepNow && m_owner->m_nbSteps == m_owner->m_nbStepsAtRelease)
    {
        m_owner->m_guiYieldsForStep = true;
        m_owner->m_simulationCondition.notify_all();
        m_owner->m_guiCondition.wait(lock, [this] { return !m_owner->m_guiYieldsForStep; });
    }

    lock.release(); // held until unlock()
}

SimulationThread::SceneLock::~SceneLock()
{
    unlock();
}

SimulationThread::SceneLock::SceneLock(SceneLock&& other) noexcept
    : m_owner(other.m_owner)
{
    other.m_owner = nullptr;
}

SimulationThread::SceneLock& SimulationThread::SceneLock::operator=(SceneLock&& other) noexcept
{
    if (this != &other)
    {
        unlock();
        m_owner = other.m_owner;
        other.m_owner = nullptr;
    }
    return *this;
}

void SimulationThread::SceneLock::unlock()
{
    if (!m_owner)
        return;
    m_owner->m_nbStepsAtRelease = m_owner->m_nbSteps;
    m_owner->m_guiRequests.fetch_sub(1);
    m_owner->m_mutex.unlock();
    m_owner->m_simulationCondition.notify_all();
    m_owner = nullptr;
}

} // namespace sofaglfw
