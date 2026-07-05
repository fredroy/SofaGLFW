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
#include <SofaGLFW/render/gl/SceneRendererGL.h>

#include <sofa/gl/gl.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/component/visual/BaseCamera.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h> // sofa::simulation::node::draw
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/SetDirectory.h>
#include <sofa/helper/logging/Messaging.h>

#include <ranges>

using namespace sofa;

namespace sofaglfw::render
{

SceneRendererGL::~SceneRendererGL()
{
    releaseResources();
}

void SceneRendererGL::drawScene(sofa::simulation::Node* groot,
                                sofa::core::visual::VisualParams* vparams,
                                sofa::component::visual::BaseCamera* camera,
                                GLFWwindow* glfwWindow,
                                const ViewportRect& viewport,
                                const sofa::type::RGBAColor& background)
{
    SOFA_UNUSED(glfwWindow);

    glClearColor(background.r(), background.g(), background.b(), background.a());
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_currentBackgroundFilename.empty())
        drawBackgroundImage(camera);

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_COLOR_MATERIAL);

    if (!camera)
    {
        msg_error("SceneRendererGL") << "No camera defined.";
        return;
    }

    if (groot->f_bbox.getValue().isValid())
    {
        vparams->sceneBBox() = groot->f_bbox.getValue();
        camera->setBoundingBox(vparams->sceneBBox().minBBox(), vparams->sceneBBox().maxBBox());
    }
    camera->computeZ();
    camera->d_widthViewport.setValue(viewport.width);
    camera->d_heightViewport.setValue(viewport.height);

    // matrices
    double lastModelviewMatrix[16];
    double lastProjectionMatrix[16];

    camera->getOpenGLProjectionMatrix(lastProjectionMatrix);
    camera->getOpenGLModelViewMatrix(lastModelviewMatrix);

    glViewport(0, 0, viewport.width, viewport.height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixd(lastProjectionMatrix);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixd(lastModelviewMatrix);

    // Update the visual params
    vparams->zNear() = camera->getZNear();
    vparams->zFar() = camera->getZFar();
    vparams->setProjectionMatrix(lastProjectionMatrix);
    vparams->setModelViewMatrix(lastModelviewMatrix);

    sofa::simulation::node::draw(vparams, groot);
}

void SceneRendererGL::setBackgroundImage(const std::string& filename)
{
    // when setting a background image, we check if it was not loaded and cached first
    if (!m_backgrounds.contains(filename))
    {
        std::string tempFilename = filename;
        if (sofa::helper::system::DataRepository.findFile(tempFilename))
        {
            const auto backgroundImageFilename = sofa::helper::system::DataRepository.getFile(tempFilename);

            std::string extension = sofa::helper::system::SetDirectory::GetExtension(filename.c_str());
            std::ranges::transform(extension, extension.begin(), ::tolower);

            auto* backgroundImage = helper::io::Image::FactoryImage::getInstance()->createObject(extension, backgroundImageFilename);
            if (!backgroundImage)
            {
                msg_warning("SceneRendererGL") << "Could not load the file " << filename;
                return;
            }
            else
            {
                auto* texture = new gl::Texture(backgroundImage);
                if (texture)
                {
                    texture->init();
                    m_backgrounds.emplace(filename, Background{ backgroundImage, texture });
                }
            }
        }
    }
    m_currentBackgroundFilename = filename;
}

void SceneRendererGL::clearBackgroundImage()
{
    m_currentBackgroundFilename.clear();
}

void SceneRendererGL::drawBackgroundImage(sofa::component::visual::BaseCamera* camera)
{
    if (!m_backgrounds.contains(m_currentBackgroundFilename))
        return;
    if (!camera)
        return;

    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);

    const auto& background = m_backgrounds[m_currentBackgroundFilename];

    if (!background.image)
    {
        glPopAttrib();
        return;
    }

    const int imageWidth = background.image->getWidth();
    const int imageHeight = background.image->getHeight();

    const int screenWidth = camera->d_widthViewport.getValue();
    const int screenHeight = camera->d_heightViewport.getValue();

    glEnable(GL_TEXTURE_2D);
    glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(-0.5, screenWidth, -0.5, screenHeight, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    background.texture->bind();

    const double coordWidth = int(screenWidth / imageWidth) + 1;
    const double coordHeight = int(screenHeight / imageHeight) + 1;

    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_QUADS);
    glTexCoord2d(0.0,            0.0);             glVertex3d( -imageWidth*coordWidth, -imageHeight*coordHeight, 0.0 );
    glTexCoord2d(coordWidth*2.0, 0.0);             glVertex3d(  imageWidth*coordWidth, -imageHeight*coordHeight, 0.0 );
    glTexCoord2d(coordWidth*2.0, coordHeight*2.0); glVertex3d(  imageWidth*coordWidth,  imageHeight*coordHeight, 0.0 );
    glTexCoord2d(0.0,            coordHeight*2.0); glVertex3d( -imageWidth*coordWidth,  imageHeight*coordHeight, 0.0 );
    glEnd();

    glBindTexture(GL_TEXTURE_2D, 0);

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glDisable(GL_TEXTURE_2D);

    glPopAttrib();
}

void SceneRendererGL::releaseResources()
{
    for (auto& [_, background] : m_backgrounds)
    {
        delete background.texture;
        // Note: the image is owned by the texture lifetime in master; deleting
        // the texture is sufficient. The Image is created by the factory and
        // handed to gl::Texture which does not take ownership, so free it too.
    }
    m_backgrounds.clear();
    m_currentBackgroundFilename.clear();
}

} // namespace sofaglfw::render
