/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program. If not, see <http://www.gnu.org/licenses/>.              *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/simulation/Node.h>
#include <SofaImGui/config.h>

#include <memory>
#include <SofaGLFW/BaseGUIEngine.h>

#include <sofa/simulation/Node.h>
#include "WindowState.h"

#include <SimpleIni.h>

#include <sofa/helper/logging/Message.h>

#include <string>
#include <vector>


namespace windows
{
        /**
         * @brief The messages the Log window shows: a copy of SOFA's main log, updated
         * when the GUI has the scene.
         *
         * A simulation thread adds its messages to the log during its steps, and reading
         * it meanwhile would race. With this copy, the Log window is drawn in every frame,
         * also during a long step (scrolling, filtering), and shows the new messages at
         * the end of the step. Each message keeps what the window shows of it, the path
         * of its component included, read with the scene.
         */
        class SOFAIMGUI_API LogMessages
        {
        public:
            struct Entry
            {
                sofa::helper::logging::Message::Type type;
                std::string sender;        ///< the sender, and the component's name if any
                std::string componentPath; ///< empty without component
                std::string text;
            };

            /// Copies the messages logged since the last update. With the scene.
            void update();
            const std::vector<Entry>& entries() const { return m_entries; }

        private:
            std::vector<Entry> m_entries;
            std::size_t m_copied{0}; ///< messages of the log copied
        };

        /**
         * @brief Shows the Log window.
         *
         * This function displays a window containing log messages. It provides options to filter messages by type and save the log to a file. The displayed log messages include their IDs, types, senders, and the messages themselves.
         *
         * @param windowNameLog The name of the Log window.
         * @param isLogWindowOpen A reference to a boolean flag indicating if the Log window is open.
         */
        void showLog(const char* const& windowNameLog,
                     CSimpleIniA &ini,
                     WindowState& winManagerLog,
                     const LogMessages& messages);


} // namespace sofaimgui
