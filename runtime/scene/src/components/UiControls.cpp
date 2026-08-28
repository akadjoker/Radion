#include "UiControls.h"

#include "GameObject.h"
#include "Input.h"
#include "ScreenDraw.h"

namespace Radion
{
namespace
{

// Regions in a UI atlas laid out the same way the reference's embedded
// menu.png is - so a project supplying that same atlas through
// UiSystem::setThemeTexture() gets the exact same widget art.
constexpr f32 kPanelX = 2.0f, kPanelY = 2.0f, kPanelW = 72.0f, kPanelH = 72.0f;
constexpr f32 kButtonX = 79.0f, kButtonY = 2.0f, kButtonW = 44.0f, kButtonH = 44.0f;
constexpr f32 kCheckOffX = 79.0f, kCheckOffY = 2.0f;
constexpr f32 kCheckOnX = 79.0f, kCheckOnY = 47.0f, kCheckW = 44.0f, kCheckH = 44.0f;
constexpr f32 kSliderTrackX = 3.0f, kSliderTrackY = 82.0f, kSliderTrackW = 71.0f, kSliderTrackH = 5.0f;
constexpr f32 kSliderKnobX = 6.0f, kSliderKnobY = 150.0f, kSliderKnobW = 9.0f, kSliderKnobH = 23.0f;

f32 MeasureUiText(const std::string& text, f32 glyphSize)
{
    f32 widest = 0.0f;
    f32 width = 0.0f;
    for (char character : text)
    {
        if (character == '\n')
        {
            widest = widest > width ? widest : width;
            width = 0.0f;
        }
        else
            width += glyphSize;
    }
    return widest > width ? widest : width;
}

// Nearest ancestor carrying any of the five widget types, or null when none
// of `start`'s ancestors carries one - the layout base is then the screen
// itself. Stands in for the reference's Component::uiControl() virtual,
// which Radion's own Component base does not carry; the widget set is
// closed and known here, so trying each concrete type directly needs no
// change to Component/GameObject at all.
UiControl* FindParentControl(GameObject* start)
{
    for (GameObject* parent = start; parent; parent = parent->parent())
    {
        if (UiPanel* control = parent->getComponent<UiPanel>())
            return control;
        if (UiLabel* control = parent->getComponent<UiLabel>())
            return control;
        if (UiButton* control = parent->getComponent<UiButton>())
            return control;
        if (UiCheckBox* control = parent->getComponent<UiCheckBox>())
            return control;
        if (UiSlider* control = parent->getComponent<UiSlider>())
            return control;
    }
    return nullptr;
}

} // namespace

// ---------------------------------------------------------------- UiSystem

UiSystem& UiSystem::getSingleton()
{
    static UiSystem instance;
    return instance;
}

UiSystem::UiSystem()
{
}

void UiSystem::setScreenSize(f32 width, f32 height)
{
    mViewport.width = width > 0.0f ? width : 0.0f;
    mViewport.height = height > 0.0f ? height : 0.0f;
    mViewport.valid = mViewport.width > 0.0f && mViewport.height > 0.0f;
}

const UiViewport& UiSystem::viewport() const
{
    return mViewport;
}

void UiSystem::setThemeTexture(TextureHandle texture)
{
    mThemeTexture = texture;
}

void UiSystem::clearThemeTexture()
{
    mThemeTexture = TextureHandle();
}

bool UiSystem::hasThemeTexture() const
{
    return mThemeTexture.valid();
}

TextureHandle UiSystem::themeTexture() const
{
    return mThemeTexture;
}

void UiSystem::registerControl(UiControl* control)
{
    if (control)
        mControls.push_back(control);
}

void UiSystem::unregisterControl(UiControl* control)
{
    for (usize i = 0; i < mControls.size(); ++i)
        if (mControls[i] == control)
        {
            if (mRefreshing)
                mControls[i] = nullptr;
            else
            {
                mControls[i] = mControls.back();
                mControls.pop_back();
            }
            return;
        }
}

s32 UiSystem::nextOrder()
{
    return mNextOrder++;
}

void UiSystem::compactControls()
{
    usize write = 0;
    for (usize read = 0; read < mControls.size(); ++read)
        if (mControls[read])
            mControls[write++] = mControls[read];
    mControls.resize(write);
}

void UiSystem::refresh()
{
    if (!mViewport.valid)
        return;

    mRefreshing = true;

    bool anyInteractive = false;
    for (usize i = 0; i < mControls.size(); ++i)
    {
        UiControl* control = mControls[i];
        GameObject* object = control ? control->owner() : nullptr;
        if (!object || object->disposed() || !object->isActiveAndVisibleInHierarchy() ||
            !control->active())
            continue;
        control->updateLayout();
        control->resetInput();
        if (control->interactive())
            anyInteractive = true;
    }

    if (anyInteractive)
    {
        const glm::vec2 mouse = Input::getMousePosition();
        if (mouse.x >= 0.0f && mouse.y >= 0.0f && mouse.x < mViewport.width &&
            mouse.y < mViewport.height)
        {
            UiControl* hit = nullptr;
            for (usize i = 0; i < mControls.size(); ++i)
            {
                UiControl* control = mControls[i];
                GameObject* object = control ? control->owner() : nullptr;
                if (!control || !object || !object->isActiveAndVisibleInHierarchy() ||
                    !control->active() || !control->interactive() ||
                    !control->contains(mouse.x, mouse.y))
                    continue;
                if (!hit || control->mLayer > hit->mLayer ||
                    (control->mLayer == hit->mLayer && control->mOrder > hit->mOrder))
                    hit = control;
            }
            if (hit)
                hit->handleInput(mouse.x, mouse.y, Input::isMouseDown(LEFT),
                                 Input::isMousePressed(LEFT), Input::isMouseReleased(LEFT));
        }
    }

    mRefreshing = false;
    compactControls();
}

// ---------------------------------------------------------------- UiCanvas

UiCanvas::UiCanvas() : Component(Type)
{
}

// ---------------------------------------------------------------- UiControl

UiControl::UiControl(ComponentType type, bool interactive)
    : Component(type, ComponentEventUpdate), mInteractive(interactive)
{
}

const glm::vec4& UiControl::anchors() const
{
    return mAnchors;
}

const glm::vec4& UiControl::offsets() const
{
    return mOffsets;
}

void UiControl::setAnchors(const glm::vec4& value)
{
    mAnchors = value;
}

void UiControl::setOffsets(const glm::vec4& value)
{
    mOffsets = value;
}

void UiControl::setRect(f32 x, f32 y, f32 width, f32 height)
{
    mAnchors = glm::vec4(0.0f);
    mOffsets = glm::vec4(x, y, x + width, y + height);
}

const FloatRect& UiControl::rect() const
{
    return mRect;
}

bool UiControl::hovered() const
{
    return mHovered;
}

bool UiControl::pressed() const
{
    return mPressed;
}

bool UiControl::clicked() const
{
    return mClicked;
}

bool UiControl::interactive() const
{
    return mInteractive;
}

void UiControl::setInteractive(bool value)
{
    mInteractive = value;
}

s32 UiControl::layer() const
{
    return mLayer;
}

void UiControl::setLayer(s32 value)
{
    mLayer = value;
}

void UiControl::onAwake()
{
    mOrder = UiSystems().nextOrder();
    UiSystems().registerControl(this);
}

void UiControl::onDestroy()
{
    UiSystems().unregisterControl(this);
}

void UiControl::onUpdate(f32)
{
    if (owner() && owner()->isActiveAndVisibleInHierarchy())
        onUiRender();
}

void UiControl::onUiRender()
{
}

void UiControl::onUiInput(bool, bool, bool)
{
}

void UiControl::updateLayout()
{
    f32 baseX = 0.0f;
    f32 baseY = 0.0f;
    f32 baseW = UiSystems().viewport().width;
    f32 baseH = UiSystems().viewport().height;
    if (UiControl* parent = FindParentControl(owner() ? owner()->parent() : nullptr))
    {
        const FloatRect& parentRect = parent->rect();
        baseX = parentRect.x;
        baseY = parentRect.y;
        baseW = parentRect.width;
        baseH = parentRect.height;
    }
    const f32 left = baseX + baseW * mAnchors.x + mOffsets.x;
    const f32 top = baseY + baseH * mAnchors.y + mOffsets.y;
    mRect = FloatRect(left, top, baseW * (mAnchors.z - mAnchors.x) + mOffsets.z - mOffsets.x,
                      baseH * (mAnchors.w - mAnchors.y) + mOffsets.w - mOffsets.y);
}

bool UiControl::contains(f32 x, f32 y) const
{
    return x >= mRect.x && y >= mRect.y && x < mRect.x + mRect.width && y < mRect.y + mRect.height;
}

void UiControl::resetInput()
{
    mHovered = false;
    mClicked = false;
}

void UiControl::handleInput(f32 x, f32 y, bool down, bool pressed, bool released)
{
    mHovered = contains(x, y);
    if (!mHovered)
    {
        if (released)
            mPressed = false;
        return;
    }
    if (pressed)
        mPressed = true;
    if (released)
    {
        mClicked = mPressed;
        mPressed = false;
    }
    onUiInput(down, pressed, released);
}

void UiControl::drawSolidRect(f32 x, f32 y, f32 width, f32 height, Color color) const
{
    ScreenDraws().rect(x, y, width, height, color, true, mLayer);
}

void UiControl::drawThemeRect(f32 x, f32 y, f32 width, f32 height, f32 srcX, f32 srcY,
                              f32 srcWidth, f32 srcHeight, Color color) const
{
    if (!UiSystems().hasThemeTexture())
        return;
    ScreenDraws().sprite(UiSystems().themeTexture(), x, y, width, height, color, srcX, srcY,
                         srcWidth, srcHeight, 0.0f, 0.0f, 0.0f, mLayer);
}

void UiControl::drawText(f32 x, f32 y, f32 size, const std::string& text, Color color) const
{
    if (text.empty())
        return;
    ScreenDraws().text(x, y, size, color, text.c_str(), mLayer);
}

// ---------------------------------------------------------------- UiPanel

UiPanel::UiPanel() : UiControl(Type, false), mColor(Color::fromRGBFloat(0.10f, 0.12f, 0.16f, 0.94f))
{
}

Color UiPanel::color() const
{
    return mColor;
}

void UiPanel::setColor(Color value)
{
    mColor = value;
}

void UiPanel::onUiRender()
{
    const FloatRect r = rect();
    if (UiSystems().hasThemeTexture())
        drawThemeRect(r.x, r.y, r.width, r.height, kPanelX, kPanelY, kPanelW, kPanelH, mColor);
    else
        drawSolidRect(r.x, r.y, r.width, r.height, mColor);
}

// ---------------------------------------------------------------- UiLabel

UiLabel::UiLabel() : UiControl(Type, false), mText("Label"), mColor(Color::White)
{
}

const std::string& UiLabel::text() const
{
    return mText;
}

void UiLabel::setText(const std::string& value)
{
    mText = value;
}

f32 UiLabel::fontSize() const
{
    return mFontSize;
}

void UiLabel::setFontSize(f32 value)
{
    mFontSize = value > 1.0f ? value : 1.0f;
}

Color UiLabel::color() const
{
    return mColor;
}

void UiLabel::setColor(Color value)
{
    mColor = value;
}

void UiLabel::onUiRender()
{
    const FloatRect r = rect();
    drawText(r.x, r.y + (r.height - mFontSize) * 0.5f, mFontSize, mText, mColor);
}

// ---------------------------------------------------------------- UiButton

UiButton::UiButton() : UiControl(Type, true), mText("Button")
{
}

const std::string& UiButton::text() const
{
    return mText;
}

void UiButton::setText(const std::string& value)
{
    mText = value;
}

bool UiButton::consumeClick()
{
    const bool value = mActivated;
    mActivated = false;
    return value;
}

void UiButton::onUiInput(bool, bool, bool released)
{
    if (released && clicked())
        mActivated = true;
}

void UiButton::onUiRender()
{
    const FloatRect r = rect();
    const Color base = pressed() ? Color::fromRGBFloat(0.18f, 0.42f, 0.72f)
                      : hovered() ? Color::fromRGBFloat(0.25f, 0.55f, 0.90f)
                                  : Color::fromRGBFloat(0.20f, 0.46f, 0.78f);
    if (UiSystems().hasThemeTexture())
        drawThemeRect(r.x, r.y, r.width, r.height, kButtonX, kButtonY, kButtonW, kButtonH, base);
    else
        drawSolidRect(r.x, r.y, r.width, r.height, base);
    constexpr f32 textSize = 16.0f;
    const f32 textWidth = MeasureUiText(mText, textSize);
    const f32 textX = r.width > textWidth ? r.x + (r.width - textWidth) * 0.5f : r.x + 8.0f;
    drawText(textX, r.y + (r.height - textSize) * 0.5f, textSize, mText, Color::White);
}

// ------------------------------------------------------------- UiCheckBox

UiCheckBox::UiCheckBox() : UiControl(Type, true), mText("CheckBox")
{
}

const std::string& UiCheckBox::text() const
{
    return mText;
}

void UiCheckBox::setText(const std::string& value)
{
    mText = value;
}

bool UiCheckBox::checked() const
{
    return mChecked;
}

void UiCheckBox::setChecked(bool value)
{
    mChecked = value;
}

bool UiCheckBox::consumeChanged()
{
    const bool value = mChanged;
    mChanged = false;
    return value;
}

void UiCheckBox::onUiInput(bool, bool, bool released)
{
    if (released && clicked())
    {
        mChecked = !mChecked;
        mChanged = true;
    }
}

void UiCheckBox::onUiRender()
{
    const FloatRect r = rect();
    const f32 side = r.height < 22.0f ? r.height : 22.0f;
    const f32 top = r.y + (r.height - side) * 0.5f;
    if (UiSystems().hasThemeTexture())
    {
        const Color tint = mChecked ? Color::fromRGBFloat(0.35f, 0.78f, 0.50f)
                                    : Color::fromRGBFloat(0.72f, 0.80f, 0.93f);
        drawThemeRect(r.x, top, side, side, mChecked ? kCheckOnX : kCheckOffX,
                     mChecked ? kCheckOnY : kCheckOffY, kCheckW, kCheckH, tint);
    }
    else
    {
        drawSolidRect(r.x, top, side, side, Color::fromRGBFloat(0.18f, 0.22f, 0.30f));
        if (mChecked)
            drawSolidRect(r.x + 5.0f, top + 5.0f, side - 10.0f, side - 10.0f,
                          Color::fromRGBFloat(0.26f, 0.75f, 0.45f));
    }
    drawText(r.x + side + 8.0f, r.y + (r.height - 16.0f) * 0.5f, 16.0f, mText, Color::White);
}

// --------------------------------------------------------------- UiSlider

UiSlider::UiSlider() : UiControl(Type, true)
{
}

f32 UiSlider::value() const
{
    return mValue;
}

void UiSlider::setValue(f32 value)
{
    mValue = value < mMinimum ? mMinimum : (value > mMaximum ? mMaximum : value);
}

f32 UiSlider::minimum() const
{
    return mMinimum;
}

f32 UiSlider::maximum() const
{
    return mMaximum;
}

void UiSlider::setRange(f32 minimum, f32 maximum)
{
    mMinimum = minimum;
    mMaximum = maximum > minimum ? maximum : minimum + 1.0f;
    setValue(mValue);
}

bool UiSlider::consumeChanged()
{
    const bool value = mChanged;
    mChanged = false;
    return value;
}

void UiSlider::onUiInput(bool down, bool pressed, bool)
{
    if (!down && !pressed)
        return;
    const FloatRect r = rect();
    const glm::vec2 mouse = Input::getMousePosition();
    const f32 t = r.width > 1.0f ? (mouse.x - r.x) / r.width : 0.0f;
    const f32 next = mMinimum + (t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t)) * (mMaximum - mMinimum);
    if (next != mValue)
    {
        mValue = next;
        mChanged = true;
    }
}

void UiSlider::onUiRender()
{
    const FloatRect r = rect();
    const f32 t = (mValue - mMinimum) / (mMaximum - mMinimum);
    if (UiSystems().hasThemeTexture())
    {
        const f32 trackY = r.y + r.height * 0.5f - kSliderTrackH * 0.5f;
        const f32 knobX = r.x + t * (r.width - kSliderKnobW);
        const f32 knobY = r.y + (r.height - kSliderKnobH) * 0.5f;
        drawThemeRect(r.x, trackY, r.width, kSliderTrackH, kSliderTrackX, kSliderTrackY,
                     kSliderTrackW, kSliderTrackH, Color::fromRGBFloat(0.20f, 0.26f, 0.36f));
        drawThemeRect(r.x, trackY, t * r.width, kSliderTrackH, kSliderTrackX, kSliderTrackY,
                     kSliderTrackW, kSliderTrackH, Color::fromRGBFloat(0.24f, 0.58f, 0.93f));
        drawThemeRect(knobX, knobY, kSliderKnobW, kSliderKnobH, kSliderKnobX, kSliderKnobY,
                     kSliderKnobW, kSliderKnobH,
                     hovered() ? Color::fromRGBFloat(0.85f, 0.90f, 1.0f)
                               : Color::fromRGBFloat(0.70f, 0.78f, 0.92f));
    }
    else
    {
        const f32 knobX = r.x + t * (r.width - 14.0f);
        drawSolidRect(r.x, r.y + r.height * 0.5f - 3.0f, r.width, 6.0f,
                      Color::fromRGBFloat(0.16f, 0.20f, 0.27f));
        drawSolidRect(r.x, r.y + r.height * 0.5f - 3.0f, t * r.width, 6.0f,
                      Color::fromRGBFloat(0.24f, 0.58f, 0.93f));
        drawSolidRect(knobX, r.y + r.height * 0.5f - 9.0f, 14.0f, 18.0f,
                      hovered() ? Color::fromRGBFloat(0.85f, 0.90f, 1.0f)
                                : Color::fromRGBFloat(0.70f, 0.78f, 0.92f));
    }
}

} // namespace Radion
