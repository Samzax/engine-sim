#ifndef ATG_ENGINE_SIM_ENGINE_VIEW_H
#define ATG_ENGINE_SIM_ENGINE_VIEW_H

#include "ui_element.h"

class Engine;

class EngineView : public UiElement {
    public:
        EngineView();
        virtual ~EngineView();

        virtual void update(float dt);
        virtual void render();
        virtual void onMouseDown(const Point &mouseLocal);
        virtual void onDrag(const Point &p0, const Point &mouse0, const Point &mouse);
        virtual void onMouseScroll(int scroll);

        void setDrawFrame(bool drawFrame) { m_drawFrame = drawFrame; }
        void setBounds(const Bounds &bounds);
        void fitEngine(Engine *engine);

        Point getCenter() const;

        Point getCameraPosition() const;
        float m_zoom;
        
    protected:
        Point m_pan;
        Point m_dragStart;
        int m_lastScroll;
        bool m_drawFrame;
        bool m_fitPending = false;
        Point m_engineMin;
        Point m_engineMax;
};

#endif /* ATG_ENGINE_SIM_ENGINE_VIEW_H */
