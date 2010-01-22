/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

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

#include "Form/Form.hpp"
#include "Form/Internal.hpp"
#include "PeriodClock.hpp"
#include "Asset.hpp"
#include "Interface.hpp"
#include "MapWindow.h"
#include "Screen/Animation.hpp"
#include "Screen/SingleWindow.hpp"
#include "Screen/Layout.hpp"

PeriodClock WndForm::timeAnyOpenClose;

WndForm::WndForm(SingleWindow &_main_window,
                 const TCHAR *Name, const TCHAR *Caption,
                 int X, int Y, int Width, int Height):
  ContainerControl(NULL, &_main_window, Name, X, Y, Width, Height, false),
  main_window(_main_window),
  mModalResult(0),
  mColorTitle(Color::YELLOW),
  mhTitleFont(GetFont()),
  mClientWindow(NULL),
  mOnTimerNotify(NULL), mOnKeyDownNotify(NULL), mOnUserMsgNotify(NULL)
{
  // Create ClientWindow
  mClientWindow = new ContainerControl(this, this,
                                       TEXT(""), 20, 20, Width, Height);
  mClientWindow->SetBackColor(GetBackColor());

  mClientRect.top=0;
  mClientRect.left=0;
  mClientRect.bottom=Width;
  mClientRect.right=Height;

  cbTimerID = set_timer(1001, 500);

  if (Caption != NULL)
    _tcscpy(mCaption, Caption);

}

WndForm::~WndForm()
{
  /* we must override the ~Window() reset call, because in ~Window(),
     our own on_destroy() method won't be called (during object
     destruction, this object loses its identity) */
  reset();
}

ContainerWindow &
WndForm::GetClientAreaWindow(void)
{

  if (mClientWindow != NULL)
    return *mClientWindow;
  else
    return *this;
}


void WndForm::AddClient(WindowControl *Client){      // add client window
  if (mClientWindow != NULL){
    mClientWindow->AddClient(Client); // add it to the clientarea window
  } else
    ContainerControl::AddClient(Client);
}

bool
WndForm::on_destroy()
{
  if (mModalResult == 0)
    mModalResult = mrCancel;

  ContainerControl::on_destroy();
  return true;
}

bool
WndForm::on_timer(timer_t id)
{
  if (id == cbTimerID) {
    if (mOnTimerNotify)
      mOnTimerNotify(this);
    return true;
  } else
    return ContainerControl::on_timer(id);
}

bool
WndForm::on_user(unsigned id)
{
  if (mOnUserMsgNotify != NULL && mOnUserMsgNotify(this, id))
    return true;

  return ContainerControl::on_user(id);
}

const Font *
WndForm::SetTitleFont(const Font &font)
{
  const Font *res = mhTitleFont;

  if (mhTitleFont != &font){
    // todo
    mhTitleFont = &font;

    invalidate();
  }

  return res;

}

int WndForm::ShowModal(void){
  return ShowModal(false);
}

#ifndef ENABLE_SDL

static bool
is_user_input(UINT message)
{
  return message == WM_KEYDOWN || message == WM_KEYUP ||
    message == WM_LBUTTONDOWN || message == WM_LBUTTONUP ||
    message == WM_LBUTTONDBLCLK;
}

static bool
is_allowed_map_message(UINT message)
{
  return message == WM_LBUTTONDOWN || message == WM_LBUTTONUP ||
    message == WM_MOUSEMOVE;
}

static bool
is_allowed_map(HWND hWnd, UINT message, bool enable_map)
{
  return !is_altair() && enable_map && MapWindow::identify(hWnd) &&
    is_allowed_map_message(message);
}

#endif /* !ENABLE_SDL */

int WndForm::ShowModal(bool bEnableMap) {
  assert_none_locked();

#define OPENCLOSESUPPRESSTIME 500
#ifndef ENABLE_SDL
  MSG msg;
  HWND oldFocusHwnd;
#endif /* !ENABLE_SDL */

  PeriodClock enter_clock;
  if (is_embedded() && !is_altair())
    enter_clock.update();

  RECT mRc = get_screen_position();
  DrawWireRects(XCSoarInterface::EnableAnimation,&mRc, 5);

  show_on_top();

  mModalResult = 0;

#ifndef ENABLE_SDL
  oldFocusHwnd = ::GetFocus();
#endif /* !ENABLE_SDL */
  set_focus();

  FocusNext(NULL);

#ifndef ENABLE_SDL
  bool hastimed = false;
#endif /* !ENABLE_SDL */
  WndForm::timeAnyOpenClose.update(); // when current dlg opens or child closes

  main_window.add_dialog(this);

#ifdef ENABLE_SDL

  update();

  SDL_Event event;
  while (SDL_WaitEvent(&event)) {
    if (event.type == SDL_QUIT)
      break;

    if (event.type >= SDL_USEREVENT && event.type <= SDL_NUMEVENTS-1 &&
        event.user.data1 != NULL) {
      Window *window = (Window *)event.user.data1;
      window->on_user(event.type - SDL_USEREVENT);
    } else
      parent->on_event(event);
  }

  main_window.remove_dialog(this);
  return 0;

#else /* !ENABLE_SDL */
  while ((mModalResult == 0) && GetMessage(&msg, NULL, 0, 0)) {
//hack!

    // JMW update display timeout so we don't get blanking
    /*
    if (msg.message == WM_KEYDOWN) {
      if (!Debounce()) {
        continue;
      }
    }
    */

    if (msg.message == WM_KEYDOWN) {
      XCSoarInterface::InterfaceTimeoutReset();
    }

    if (is_user_input(msg.message)
        && !identify_descendant(msg.hwnd) // not current window or child
        && !is_allowed_map(msg.hwnd, msg.message, bEnableMap))
      continue;   // make it modal

    // hack to stop exiting immediately
    if (is_embedded() && !is_altair() && !hastimed &&
        is_user_input(msg.message)) {
      if (!enter_clock.check(1000))
        /* ignore user input in the first 1000ms */
        continue;
      else
        hastimed = true;
    }

    if (msg.message == WM_KEYDOWN && mOnKeyDownNotify != NULL &&
        mOnKeyDownNotify(this, msg.wParam))
      continue;

    TranslateMessage(&msg);
    if (msg.message != WM_LBUTTONUP ||
        // prevents child click from being repeat-handled by parent
        // if buttons overlap
        WndForm::timeAnyOpenClose.elapsed() > OPENCLOSESUPPRESSTIME) {
      assert_none_locked();

      DispatchMessage(&msg);

      assert_none_locked();
    }
  } // End Modal Loop
#endif /* !ENABLE_SDL */

  main_window.remove_dialog(this);

  // static.  this is current open/close or child open/close
  WndForm::timeAnyOpenClose.update();

  //  SetSourceRectangle(mRc);
  //  DrawWireRects(&aniRect, 5);

  /*
  // reset to center?
  aniRect.top = (mRc.top+mRc.bottom)/2;;
  aniRect.left = (mRc.left+mRc.right)/2;
  aniRect.right = (mRc.left+mRc.right)/2;
  aniRect.bottom = (mRc.top+mRc.bottom)/2;
  SetSourceRectangle(aniRect);
  */

#ifndef ENABLE_SDL
  SetFocus(oldFocusHwnd);
#endif /* !ENABLE_SDL */

  return mModalResult;
}

void
WndForm::on_paint(Canvas &canvas)
{
  ContainerControl::on_paint(canvas);

  // Get window coordinates
  RECT rcClient = get_client_rect();

  // Select default pen and brush
  canvas.select(GetBorderPen());
  canvas.select(GetBackBrush());

  // Draw the borders
  canvas.raised_edge(rcClient);

  // Set the colors
  canvas.set_text_color(GetForeColor());
  canvas.set_background_color(mColorTitle);
  canvas.background_transparent();

  // Set the titlebar font and font-size
  canvas.select(*mhTitleFont);
  SIZE tsize = canvas.text_size(mCaption);

  // JMW todo add here icons?

  // Calculate the titlebar coordinates
  CopyRect(&mTitleRect, &rcClient);
  mTitleRect.bottom = mTitleRect.top;
  if (tsize.cy > 0)
    mTitleRect.bottom += tsize.cy + Layout::FastScale(1);

  if (mClientWindow && !EqualRect(&mClientRect, &rcClient)) {
    // Calculate the ClientWindow coordinates
    rcClient.top += tsize.cy + Layout::FastScale(1);

    // Move the ClientWindow to the new coordinates
    mClientWindow->move(rcClient.left, rcClient.top,
        (rcClient.right - rcClient.left), (rcClient.bottom - rcClient.top));

    // Save the new coordinates
    CopyRect(&mClientRect, &rcClient);
  }

  // Draw titlebar text
  canvas.text_opaque(mTitleRect.left + Layout::FastScale(2),
      mTitleRect.top, &mTitleRect, mCaption);
}

void WndForm::SetCaption(const TCHAR *Value){
  if (Value == NULL)
    Value = TEXT("");

  if (_tcscmp(mCaption, Value) != 0){
    _tcscpy(mCaption, Value);
    invalidate(mTitleRect);
  }

}

Color WndForm::SetForeColor(Color Value)
{
  if (mClientWindow)
    mClientWindow->SetForeColor(Value);
  return ContainerControl::SetForeColor(Value);
}

Color WndForm::SetBackColor(Color Value)
{
  if (mClientWindow)
  mClientWindow->SetBackColor(Value);
  return ContainerControl::SetBackColor(Value);
}

const Font *WndForm::SetFont(const Font &Value){
  if (mClientWindow)
    mClientWindow->SetFont(Value);
  return ContainerControl::SetFont(Value);
}

void
WndForm::SetKeyDownNotify(KeyDownNotifyCallback_t KeyDownNotify)
{
  mOnKeyDownNotify = KeyDownNotify;
}

void
WndForm::SetTimerNotify(TimerNotifyCallback_t OnTimerNotify)
{
  mOnTimerNotify = OnTimerNotify;
}

void
WndForm::SetUserMsgNotify(UserMsgNotifyCallback_t OnUserMsgNotify)
{
  mOnUserMsgNotify = OnUserMsgNotify;
}

// normal form stuff (nonmodal)

bool
WndForm::on_unhandled_key(unsigned key_code)
{
  if (mOnKeyDownNotify != NULL && mOnKeyDownNotify(this, key_code))
    return true;

  switch (key_code) {
  case VK_ESCAPE:
    SetModalResult(mrCancel);
    return true;
  }

  return ContainerControl::on_unhandled_key(key_code);
}
