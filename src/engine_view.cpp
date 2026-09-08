#include "../include/engine_view.h"

#include "../include/engine_sim_application.h"

EngineView::EngineView() {
    m_pan = { 0, units::distance(-6, units::inch) };
    m_checkMouse = true;
    m_lastScroll = 0;
    m_zoom = 1.0f;
    m_drawFrame = true;
}

EngineView::~EngineView() {
    /* void */
}

void EngineView::update(float dt) {
    m_mouseBounds = m_bounds;
}

void EngineView::render() {
    if (m_drawFrame) {
        drawFrame(m_bounds, 1.0f, m_app->getForegroundColor(), m_app->getBackgroundColor(), false);
    }
}

void EngineView::onMouseDown(const Point &mouseLocal) {
    UiElement::onMouseDown(mouseLocal);
    m_dragStart = m_pan;
}

void EngineView::onDrag(const Point &p0, const Point &mouse0, const Point &mouse) {
    const Point delta = mouse - mouse0;
    const Point deltaUnits = {
        m_app->pixelsToUnits(delta.x),
        m_app->pixelsToUnits(delta.y)
    };

    m_pan = m_dragStart + deltaUnits;
}

void EngineView::onMouseScroll(int scroll) {
    const float f = std::powf(2.0, (float)scroll / 500.0f);

    const Point prevCenter = getCenter();

    m_zoom *= f;
    const Point newCenter = getCenter();

    Point diff = newCenter - prevCenter;
    m_pan += diff * m_zoom;
    m_dragStart += diff * m_zoom;
}

void EngineView::setBounds(const Bounds &bounds) {
    m_bounds = bounds;
    if (m_fitPending && bounds.width() > 0 && bounds.height() > 0) {
        const float aspect = bounds.width() / bounds.height();
        const float height = 1.15f * std::fmax(
            m_engineMax.y - m_engineMin.y, (m_engineMax.x - m_engineMin.x) / aspect);
        if (std::isfinite(height) && height > 0) {
            m_zoom = m_app->pixelsToUnits(bounds.height()) / height;
            const Point center = (m_engineMin + m_engineMax) * 0.5f;
            m_pan = center * -m_zoom;
        }
        m_fitPending = false;
    }
}

void EngineView::fitEngine(Engine *engine) {
    if (engine == nullptr) return;
    m_engineMin = m_engineMax = { 0, 0 };
    const auto includePoint = [this](double x, double y) {
        m_engineMin.x = std::fmin(m_engineMin.x, static_cast<float>(x));
        m_engineMin.y = std::fmin(m_engineMin.y, static_cast<float>(y));
        m_engineMax.x = std::fmax(m_engineMax.x, static_cast<float>(x));
        m_engineMax.y = std::fmax(m_engineMax.y, static_cast<float>(y));
    };
    for (int i = 0; i < engine->getCrankshaftCount(); ++i) {
        const auto *crank = engine->getCrankshaft(i);
        const double radius = 2.0 * std::abs(crank->getThrow());
        includePoint(crank->getPosX() - radius, crank->getPosY() - radius);
        includePoint(crank->getPosX() + radius, crank->getPosY() + radius);
    }
    for (int i = 0; i < engine->getCylinderBankCount(); ++i) {
        const auto *bank = engine->getCylinderBank(i);
        const double bore = bank->getBore();
        const double chamberHeight = engine->getHead(i)->getCombustionChamberVolume()
            / bank->boreSurfaceArea();
        // Include room for the head, valve train and cam drawings above the deck.
        for (double height : { 0.0, bank->getDeckHeight() + chamberHeight + 1.5 * bore }) {
            for (double side : { -1.2 * bore, 1.2 * bore }) {
                includePoint(bank->getX() + bank->getDx() * height - bank->getDy() * side,
                    bank->getY() + bank->getDy() * height + bank->getDx() * side);
            }
        }
    }
    m_fitPending = true;
}

Point EngineView::getCenter() const {
    return getCameraPosition();
}

Point EngineView::getCameraPosition() const {
    return Point(-m_pan / m_zoom);
}
