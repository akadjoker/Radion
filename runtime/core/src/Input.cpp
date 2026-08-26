#include "PCH.h"

/** ****************************************************************************
  Radion Platform - Input implementation (SDL2 backend).
**************************************************************************** */
#include "Input.h"

#include <SDL2/SDL.h>
#include <cstring>

namespace Radion
{

namespace
{
MouseButton ConvertSDLButton(Uint8 button);
KeyCode ConvertSDLScancode(SDL_Scancode scancode);
} // namespace

bool Input::currentMouseState[Input::MAX_MOUSE_BUTTONS];
bool Input::previousMouseState[Input::MAX_MOUSE_BUTTONS];
glm::vec2 Input::mousePosition;
glm::vec2 Input::mousePreviousPosition;
glm::vec2 Input::mouseOffset;
glm::vec2 Input::mouseScale(1.0f, 1.0f);
glm::vec2 Input::mouseWheel;
SDL_Cursor* Input::mouseCursor = nullptr;
MouseCursor Input::currentCursor = DEFAULT;

bool Input::currentKeyState[Input::MAX_KEYBOARD_KEYS];
bool Input::previousKeyState[Input::MAX_KEYBOARD_KEYS];
KeyCode Input::keyPressedQueue[Input::MAX_KEY_PRESSED_QUEUE];
int Input::keyPressedQueueCount = 0;
int Input::charPressedQueue[Input::MAX_CHAR_PRESSED_QUEUE];
int Input::charPressedQueueCount = 0;

SDL_GameController* Input::gamepads[Input::MAX_GAMEPADS];
bool Input::currentGamepadButtonState[Input::MAX_GAMEPADS][Input::MAX_GAMEPAD_BUTTONS];
bool Input::previousGamepadButtonState[Input::MAX_GAMEPADS][Input::MAX_GAMEPAD_BUTTONS];
float Input::gamepadAxisState[Input::MAX_GAMEPADS][Input::MAX_GAMEPAD_AXIS];
int Input::lastButtonPressed = -1;

glm::vec2 Input::touchPosition[Input::MAX_TOUCH_POINTS];
SDL_FingerID Input::touchId[Input::MAX_TOUCH_POINTS];
int Input::touchPointCount = 0;

static glm::vec2 sMouseDeltaAccum;

// ------------------------------------------------------------------------
// Lifecycle
// ------------------------------------------------------------------------

void Input::init()
{
    std::memset(currentMouseState, 0, sizeof(currentMouseState));
    std::memset(previousMouseState, 0, sizeof(previousMouseState));
    std::memset(currentKeyState, 0, sizeof(currentKeyState));
    std::memset(previousKeyState, 0, sizeof(previousKeyState));
    std::memset(currentGamepadButtonState, 0, sizeof(currentGamepadButtonState));
    std::memset(previousGamepadButtonState, 0, sizeof(previousGamepadButtonState));
    std::memset(gamepadAxisState, 0, sizeof(gamepadAxisState));
    for (int i = 0; i < MAX_GAMEPADS; ++i)
        gamepads[i] = nullptr;

    keyPressedQueueCount = 0;
    charPressedQueueCount = 0;
    mouseWheel = glm::vec2();
    mouseOffset = glm::vec2();
    mouseScale = glm::vec2(1.0f, 1.0f);
    sMouseDeltaAccum = glm::vec2();

    int numJoysticks = SDL_NumJoysticks();
    for (int i = 0; i < numJoysticks && i < MAX_GAMEPADS; ++i)
    {
        if (SDL_IsGameController(i))
            gamepads[i] = SDL_GameControllerOpen(i);
    }

    std::memset(touchPosition, 0, sizeof(touchPosition));
    std::memset(touchId, 0, sizeof(touchId));
    touchPointCount = 0;
}

void Input::update()
{
    std::memcpy(previousMouseState, currentMouseState, sizeof(currentMouseState));
    std::memcpy(previousKeyState, currentKeyState, sizeof(currentKeyState));
    std::memcpy(previousGamepadButtonState, currentGamepadButtonState,
                sizeof(currentGamepadButtonState));

    mousePreviousPosition = mousePosition;
    mouseWheel = glm::vec2();
    sMouseDeltaAccum = glm::vec2();
    keyPressedQueueCount = 0;
    charPressedQueueCount = 0;

    for (int i = 0; i < MAX_GAMEPADS; ++i)
    {
        if (!gamepads[i])
            continue;
        for (int b = 0; b < MAX_GAMEPAD_BUTTONS; ++b)
            currentGamepadButtonState[i][b] =
                SDL_GameControllerGetButton(gamepads[i],
                                            static_cast<SDL_GameControllerButton>(b)) != 0;
        for (int a = 0; a < MAX_GAMEPAD_AXIS; ++a)
            gamepadAxisState[i][a] =
                SDL_GameControllerGetAxis(gamepads[i], static_cast<SDL_GameControllerAxis>(a)) /
                32767.0f;
    }
}

// ------------------------------------------------------------------------
// Event feed (called from the platform event pump)
// ------------------------------------------------------------------------

void Input::onMouseDown(const SDL_MouseButtonEvent& event)
{
    MouseButton b = ConvertSDLButton(event.button);
    currentMouseState[b] = true;
}

void Input::onMouseUp(const SDL_MouseButtonEvent& event)
{
    MouseButton b = ConvertSDLButton(event.button);
    currentMouseState[b] = false;
}

void Input::onMouseMove(const SDL_MouseMotionEvent& event)
{
    mousePosition = glm::vec2(static_cast<float>(event.x), static_cast<float>(event.y));
    sMouseDeltaAccum.x += static_cast<float>(event.xrel);
    sMouseDeltaAccum.y += static_cast<float>(event.yrel);
}

void Input::onMouseWheel(const SDL_MouseWheelEvent& event)
{
    mouseWheel.x += static_cast<float>(event.x);
    mouseWheel.y += static_cast<float>(event.y);
}

void Input::onKeyDown(const SDL_KeyboardEvent& event)
{
    KeyCode key = ConvertSDLScancode(event.keysym.scancode);
    if (key == KEY_NULL)
        return;
    if (!currentKeyState[key] && keyPressedQueueCount < MAX_KEY_PRESSED_QUEUE)
        keyPressedQueue[keyPressedQueueCount++] = key;
    currentKeyState[key] = true;
}

void Input::onKeyUp(const SDL_KeyboardEvent& event)
{
    KeyCode key = ConvertSDLScancode(event.keysym.scancode);
    if (key == KEY_NULL)
        return;
    currentKeyState[key] = false;
}

void Input::onTextInput(const SDL_TextInputEvent& event)
{
    for (const char* c = event.text; *c && charPressedQueueCount < MAX_CHAR_PRESSED_QUEUE; ++c)
        charPressedQueue[charPressedQueueCount++] = static_cast<unsigned char>(*c);
}

void Input::onTouchDown(const SDL_TouchFingerEvent& event)
{
    if (touchPointCount >= MAX_TOUCH_POINTS)
        return;
    touchId[touchPointCount] = event.fingerId;
    touchPosition[touchPointCount] = glm::vec2(event.x, event.y);
    ++touchPointCount;
}

void Input::onTouchUp(const SDL_TouchFingerEvent& event)
{
    for (int i = 0; i < touchPointCount; ++i)
    {
        if (touchId[i] != event.fingerId)
            continue;
        // Swap-remove: touch point order isn't meaningful, so this avoids
        // shifting the rest of the (tiny) array down by one.
        touchId[i] = touchId[touchPointCount - 1];
        touchPosition[i] = touchPosition[touchPointCount - 1];
        --touchPointCount;
        break;
    }
}

void Input::onTouchMove(const SDL_TouchFingerEvent& event)
{
    for (int i = 0; i < touchPointCount; ++i)
    {
        if (touchId[i] != event.fingerId)
            continue;
        touchPosition[i] = glm::vec2(event.x, event.y);
        break;
    }
}

// ------------------------------------------------------------------------
// Touch queries
// ------------------------------------------------------------------------

int Input::getTouchPointCount()
{
    return touchPointCount;
}

glm::vec2 Input::getTouchPosition(int index)
{
    if (index < 0 || index >= touchPointCount)
        return glm::vec2();
    return touchPosition[index];
}

long long Input::getTouchPointId(int index)
{
    if (index < 0 || index >= touchPointCount)
        return -1;
    return static_cast<long long>(touchId[index]);
}

// ------------------------------------------------------------------------
// Mouse queries
// ------------------------------------------------------------------------

bool Input::isMousePressed(MouseButton button)
{
    return currentMouseState[button] && !previousMouseState[button];
}

bool Input::isMouseDown(MouseButton button)
{
    return currentMouseState[button];
}

bool Input::isMouseReleased(MouseButton button)
{
    return !currentMouseState[button] && previousMouseState[button];
}

bool Input::isMouseUp(MouseButton button)
{
    return !currentMouseState[button];
}

glm::vec2 Input::getMousePosition()
{
    return glm::vec2((mousePosition.x - mouseOffset.x) * mouseScale.x,
                     (mousePosition.y - mouseOffset.y) * mouseScale.y);
}

glm::vec2 Input::getMouseDelta()
{
    return glm::vec2(sMouseDeltaAccum.x * mouseScale.x, sMouseDeltaAccum.y * mouseScale.y);
}

int Input::getMouseX()
{
    return static_cast<int>(getMousePosition().x);
}

int Input::getMouseY()
{
    return static_cast<int>(getMousePosition().y);
}

void Input::setMousePosition(int x, int y)
{
    SDL_WarpMouseInWindow(SDL_GetMouseFocus(), x, y);
    mousePosition = glm::vec2(static_cast<float>(x), static_cast<float>(y));
}

void Input::setMouseOffset(int offsetX, int offsetY)
{
    mouseOffset = glm::vec2(static_cast<float>(offsetX), static_cast<float>(offsetY));
}

void Input::setMouseScale(float scaleX, float scaleY)
{
    mouseScale = glm::vec2(scaleX, scaleY);
}

glm::vec2 Input::getMouseWheelMove()
{
    return mouseWheel;
}

float Input::getMouseWheelMoveV()
{
    return mouseWheel.y;
}

void Input::setMouseCursor(MouseCursor cursor)
{
    static const SDL_SystemCursor map[] = {
        SDL_SYSTEM_CURSOR_ARROW,     // DEFAULT
        SDL_SYSTEM_CURSOR_ARROW,     // ARROW
        SDL_SYSTEM_CURSOR_IBEAM,     // IBEAM
        SDL_SYSTEM_CURSOR_CROSSHAIR, // CROSSHAIR
        SDL_SYSTEM_CURSOR_HAND,      // POINTING_HAND
        SDL_SYSTEM_CURSOR_SIZEWE,    // RESIZE_EW
        SDL_SYSTEM_CURSOR_SIZENS,    // RESIZE_NS
        SDL_SYSTEM_CURSOR_SIZENWSE,  // RESIZE_NWSE
        SDL_SYSTEM_CURSOR_SIZENESW,  // RESIZE_NESW
        SDL_SYSTEM_CURSOR_SIZEALL,   // RESIZE_ALL
        SDL_SYSTEM_CURSOR_NO         // NOT_ALLOWED
    };

    if (mouseCursor)
    {
        SDL_FreeCursor(mouseCursor);
        mouseCursor = nullptr;
    }
    mouseCursor = SDL_CreateSystemCursor(map[cursor]);
    if (mouseCursor)
        SDL_SetCursor(mouseCursor);
    currentCursor = cursor;
}

// ------------------------------------------------------------------------
// Keyboard queries
// ------------------------------------------------------------------------

bool Input::isKeyPressed(KeyCode key)
{
    return key < MAX_KEYBOARD_KEYS && currentKeyState[key] && !previousKeyState[key];
}

bool Input::isKeyDown(KeyCode key)
{
    return key < MAX_KEYBOARD_KEYS && currentKeyState[key];
}

bool Input::isKeyReleased(KeyCode key)
{
    return key < MAX_KEYBOARD_KEYS && !currentKeyState[key] && previousKeyState[key];
}

bool Input::isKeyUp(KeyCode key)
{
    return key >= MAX_KEYBOARD_KEYS || !currentKeyState[key];
}

KeyCode Input::getKeyPressed()
{
    if (keyPressedQueueCount == 0)
        return KEY_NULL;
    KeyCode key = keyPressedQueue[0];
    for (int i = 1; i < keyPressedQueueCount; ++i)
        keyPressedQueue[i - 1] = keyPressedQueue[i];
    --keyPressedQueueCount;
    return key;
}

int Input::getCharPressed()
{
    if (charPressedQueueCount == 0)
        return 0;
    int c = charPressedQueue[0];
    for (int i = 1; i < charPressedQueueCount; ++i)
        charPressedQueue[i - 1] = charPressedQueue[i];
    --charPressedQueueCount;
    return c;
}

// ------------------------------------------------------------------------
// Gamepad queries
// ------------------------------------------------------------------------

bool Input::isGamepadAvailable(int gamepad)
{
    return gamepad >= 0 && gamepad < MAX_GAMEPADS && gamepads[gamepad] != nullptr;
}

const char* Input::getGamepadName(int gamepad)
{
    if (!isGamepadAvailable(gamepad))
        return nullptr;
    return SDL_GameControllerName(gamepads[gamepad]);
}

bool Input::isGamepadButtonPressed(int gamepad, GamepadButton button)
{
    if (!isGamepadAvailable(gamepad))
        return false;
    return currentGamepadButtonState[gamepad][button] &&
           !previousGamepadButtonState[gamepad][button];
}

bool Input::isGamepadButtonDown(int gamepad, GamepadButton button)
{
    if (!isGamepadAvailable(gamepad))
        return false;
    return currentGamepadButtonState[gamepad][button];
}

bool Input::isGamepadButtonReleased(int gamepad, GamepadButton button)
{
    if (!isGamepadAvailable(gamepad))
        return false;
    return !currentGamepadButtonState[gamepad][button] &&
           previousGamepadButtonState[gamepad][button];
}

bool Input::isGamepadButtonUp(int gamepad, GamepadButton button)
{
    if (!isGamepadAvailable(gamepad))
        return true;
    return !currentGamepadButtonState[gamepad][button];
}

int Input::getGamepadButtonPressed()
{
    return lastButtonPressed;
}

int Input::getGamepadAxisCount(int gamepad)
{
    if (!isGamepadAvailable(gamepad))
        return 0;
    return MAX_GAMEPAD_AXIS;
}

float Input::getGamepadAxisMovement(int gamepad, GamepadAxis axis)
{
    if (!isGamepadAvailable(gamepad))
        return 0.0f;
    return gamepadAxisState[gamepad][axis];
}

// ------------------------------------------------------------------------
// Helpers - pure SDL enum conversions, kept out of the class (and out of
// Input.h) since they touch no Input state and would otherwise force the
// public header to name SDL_Scancode/SDL_GameControllerButton, which can't
// be forward-declared without pulling in <SDL2/SDL.h>.
// ------------------------------------------------------------------------

namespace
{

MouseButton ConvertSDLButton(Uint8 button)
{
    switch (button)
    {
    case SDL_BUTTON_LEFT:
        return LEFT;
    case SDL_BUTTON_RIGHT:
        return RIGHT;
    case SDL_BUTTON_MIDDLE:
        return MIDDLE;
    default:
        return LEFT;
    }
}

KeyCode ConvertSDLScancode(SDL_Scancode scancode)
{
    switch (scancode)
    {
    case SDL_SCANCODE_A:
        return KEY_A;
    case SDL_SCANCODE_B:
        return KEY_B;
    case SDL_SCANCODE_C:
        return KEY_C;
    case SDL_SCANCODE_D:
        return KEY_D;
    case SDL_SCANCODE_E:
        return KEY_E;
    case SDL_SCANCODE_F:
        return KEY_F;
    case SDL_SCANCODE_G:
        return KEY_G;
    case SDL_SCANCODE_H:
        return KEY_H;
    case SDL_SCANCODE_I:
        return KEY_I;
    case SDL_SCANCODE_J:
        return KEY_J;
    case SDL_SCANCODE_K:
        return KEY_K;
    case SDL_SCANCODE_L:
        return KEY_L;
    case SDL_SCANCODE_M:
        return KEY_M;
    case SDL_SCANCODE_N:
        return KEY_N;
    case SDL_SCANCODE_O:
        return KEY_O;
    case SDL_SCANCODE_P:
        return KEY_P;
    case SDL_SCANCODE_Q:
        return KEY_Q;
    case SDL_SCANCODE_R:
        return KEY_R;
    case SDL_SCANCODE_S:
        return KEY_S;
    case SDL_SCANCODE_T:
        return KEY_T;
    case SDL_SCANCODE_U:
        return KEY_U;
    case SDL_SCANCODE_V:
        return KEY_V;
    case SDL_SCANCODE_W:
        return KEY_W;
    case SDL_SCANCODE_X:
        return KEY_X;
    case SDL_SCANCODE_Y:
        return KEY_Y;
    case SDL_SCANCODE_Z:
        return KEY_Z;
    case SDL_SCANCODE_0:
        return KEY_ZERO;
    case SDL_SCANCODE_1:
        return KEY_ONE;
    case SDL_SCANCODE_2:
        return KEY_TWO;
    case SDL_SCANCODE_3:
        return KEY_THREE;
    case SDL_SCANCODE_4:
        return KEY_FOUR;
    case SDL_SCANCODE_5:
        return KEY_FIVE;
    case SDL_SCANCODE_6:
        return KEY_SIX;
    case SDL_SCANCODE_7:
        return KEY_SEVEN;
    case SDL_SCANCODE_8:
        return KEY_EIGHT;
    case SDL_SCANCODE_9:
        return KEY_NINE;
    case SDL_SCANCODE_SPACE:
        return KEY_SPACE;
    case SDL_SCANCODE_ESCAPE:
        return KEY_ESCAPE;
    case SDL_SCANCODE_RETURN:
        return KEY_ENTER;
    case SDL_SCANCODE_TAB:
        return KEY_TAB;
    case SDL_SCANCODE_BACKSPACE:
        return KEY_BACKSPACE;
    case SDL_SCANCODE_INSERT:
        return KEY_INSERT;
    case SDL_SCANCODE_DELETE:
        return KEY_DELETE;
    case SDL_SCANCODE_RIGHT:
        return KEY_RIGHT;
    case SDL_SCANCODE_LEFT:
        return KEY_LEFT;
    case SDL_SCANCODE_DOWN:
        return KEY_DOWN;
    case SDL_SCANCODE_UP:
        return KEY_UP;
    case SDL_SCANCODE_PAGEUP:
        return KEY_PAGE_UP;
    case SDL_SCANCODE_PAGEDOWN:
        return KEY_PAGE_DOWN;
    case SDL_SCANCODE_HOME:
        return KEY_HOME;
    case SDL_SCANCODE_END:
        return KEY_END;
    case SDL_SCANCODE_CAPSLOCK:
        return KEY_CAPS_LOCK;
    case SDL_SCANCODE_SCROLLLOCK:
        return KEY_SCROLL_LOCK;
    case SDL_SCANCODE_NUMLOCKCLEAR:
        return KEY_NUM_LOCK;
    case SDL_SCANCODE_PRINTSCREEN:
        return KEY_PRINT_SCREEN;
    case SDL_SCANCODE_PAUSE:
        return KEY_PAUSE;
    case SDL_SCANCODE_F1:
        return KEY_F1;
    case SDL_SCANCODE_F2:
        return KEY_F2;
    case SDL_SCANCODE_F3:
        return KEY_F3;
    case SDL_SCANCODE_F4:
        return KEY_F4;
    case SDL_SCANCODE_F5:
        return KEY_F5;
    case SDL_SCANCODE_F6:
        return KEY_F6;
    case SDL_SCANCODE_F7:
        return KEY_F7;
    case SDL_SCANCODE_F8:
        return KEY_F8;
    case SDL_SCANCODE_F9:
        return KEY_F9;
    case SDL_SCANCODE_F10:
        return KEY_F10;
    case SDL_SCANCODE_F11:
        return KEY_F11;
    case SDL_SCANCODE_F12:
        return KEY_F12;
    case SDL_SCANCODE_LSHIFT:
        return KEY_LEFT_SHIFT;
    case SDL_SCANCODE_LCTRL:
        return KEY_LEFT_CONTROL;
    case SDL_SCANCODE_LALT:
        return KEY_LEFT_ALT;
    case SDL_SCANCODE_LGUI:
        return KEY_LEFT_SUPER;
    case SDL_SCANCODE_RSHIFT:
        return KEY_RIGHT_SHIFT;
    case SDL_SCANCODE_RCTRL:
        return KEY_RIGHT_CONTROL;
    case SDL_SCANCODE_RALT:
        return KEY_RIGHT_ALT;
    case SDL_SCANCODE_RGUI:
        return KEY_RIGHT_SUPER;
    case SDL_SCANCODE_APOSTROPHE:
        return KEY_APOSTROPHE;
    case SDL_SCANCODE_COMMA:
        return KEY_COMMA;
    case SDL_SCANCODE_MINUS:
        return KEY_MINUS;
    case SDL_SCANCODE_PERIOD:
        return KEY_PERIOD;
    case SDL_SCANCODE_SLASH:
        return KEY_SLASH;
    case SDL_SCANCODE_SEMICOLON:
        return KEY_SEMICOLON;
    case SDL_SCANCODE_EQUALS:
        return KEY_EQUAL;
    case SDL_SCANCODE_LEFTBRACKET:
        return KEY_LEFT_BRACKET;
    case SDL_SCANCODE_BACKSLASH:
        return KEY_BACKSLASH;
    case SDL_SCANCODE_RIGHTBRACKET:
        return KEY_RIGHT_BRACKET;
    case SDL_SCANCODE_GRAVE:
        return KEY_GRAVE;
    case SDL_SCANCODE_KP_0:
        return KEY_KP_0;
    case SDL_SCANCODE_KP_1:
        return KEY_KP_1;
    case SDL_SCANCODE_KP_2:
        return KEY_KP_2;
    case SDL_SCANCODE_KP_3:
        return KEY_KP_3;
    case SDL_SCANCODE_KP_4:
        return KEY_KP_4;
    case SDL_SCANCODE_KP_5:
        return KEY_KP_5;
    case SDL_SCANCODE_KP_6:
        return KEY_KP_6;
    case SDL_SCANCODE_KP_7:
        return KEY_KP_7;
    case SDL_SCANCODE_KP_8:
        return KEY_KP_8;
    case SDL_SCANCODE_KP_9:
        return KEY_KP_9;
    case SDL_SCANCODE_KP_DECIMAL:
        return KEY_KP_DECIMAL;
    case SDL_SCANCODE_KP_DIVIDE:
        return KEY_KP_DIVIDE;
    case SDL_SCANCODE_KP_MULTIPLY:
        return KEY_KP_MULTIPLY;
    case SDL_SCANCODE_KP_MINUS:
        return KEY_KP_SUBTRACT;
    case SDL_SCANCODE_KP_PLUS:
        return KEY_KP_ADD;
    case SDL_SCANCODE_KP_ENTER:
        return KEY_KP_ENTER;
    case SDL_SCANCODE_KP_EQUALS:
        return KEY_KP_EQUAL;
    case SDL_SCANCODE_MENU:
        return KEY_KB_MENU;
    default:
        return KEY_NULL;
    }
}

} // namespace

} // namespace Radion
