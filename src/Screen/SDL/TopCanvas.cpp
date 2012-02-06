/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2012 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Screen/SDL/TopCanvas.hpp"
#include "Screen/Features.hpp"
#include "Asset.hpp"

#ifdef ENABLE_OPENGL
#include "Screen/OpenGL/Init.hpp"
#include "Screen/OpenGL/Features.hpp"
#endif

#ifdef HAVE_EGL
#include "Screen/OpenGL/EGL.hpp"
#include "Screen/OpenGL/Globals.hpp"
#endif

#ifdef ANDROID
#include "Android/Main.hpp"
#include "Android/NativeView.hpp"
#endif

#include <assert.h>

void
TopCanvas::Set(UPixelScalar width, UPixelScalar height)
{
#ifndef ANDROID
  Uint32 flags = SDL_ANYFORMAT;
#endif

#ifdef ENABLE_OPENGL
#ifndef ANDROID
  flags |= SDL_OPENGL;
#endif
#else /* !ENABLE_OPENGL */
  /* we need async screen updates as long as we don't have a global
     frame rate */
  flags |= SDL_ASYNCBLIT;

  const SDL_VideoInfo *info = SDL_GetVideoInfo();
  assert(info != NULL);

  if (info->hw_available)
    flags |= SDL_HWSURFACE;
  else
    flags |= SDL_SWSURFACE;
#endif /* !ENABLE_OPENGL */

  if (IsEmbedded()) {
#ifndef ANDROID
    flags |= SDL_FULLSCREEN;

    /* select a full-screen video mode */
    SDL_Rect **modes = SDL_ListModes(NULL, flags);
    if (modes == NULL)
      return;

    width = modes[0]->w;
    height = modes[0]->h;
#endif
  }

#ifndef ANDROID
  SDL_Surface *s = ::SDL_SetVideoMode(width, height, 0, flags);
  if (s == NULL)
    return;
#endif

#ifdef ENABLE_OPENGL
  OpenGL::SetupContext();
  OpenGL::SetupViewport(width, height);
  Canvas::set(width, height);
#else
  Canvas::set(s);
#endif
}

void
TopCanvas::OnResize(UPixelScalar width, UPixelScalar height)
{
#ifdef ENABLE_OPENGL
  OpenGL::SetupViewport(width, height);
  Canvas::set(width, height);
#endif
}

void
TopCanvas::Fullscreen()
{
#if 0 /* disabled for now, for easier development */
  ::SDL_WM_ToggleFullScreen(surface);
#endif
}

void
TopCanvas::Flip()
{
#ifdef HAVE_EGL
  if (OpenGL::egl) {
    /* if native EGL support was detected, we can circumvent the JNI
       call */
    EGLSwapBuffers();
    return;
  }
#endif

#ifdef ANDROID
  native_view->swap();
#elif defined(ENABLE_OPENGL)
  ::SDL_GL_SwapBuffers();
#else
  ::SDL_Flip(surface);
#endif
}
