#ifndef __HELLOWORLD_SCENE_H__
#define __HELLOWORLD_SCENE_H__

#include "cocos2d.h"
#include "Box2D/Box2D.h"
#include "GLES-Render.h"
#include "extensions/cocos-ext.h"
#include "ui/CocosGUI.h"

USING_NS_CC;
USING_NS_CC_EXT;
using namespace cocos2d::ui;


class HelloWorld : public cocos2d::Layer
{
public:

	HelloWorld():_label(nullptr),_isDelete(false){}


    // there's no 'id' in cpp, so we recommend returning the class instance pointer
    static cocos2d::Scene* createScene();

    // Here's a difference. Method 'init' in cocos2d-x returns bool, instead of returning 'id' in cocos2d-iphone
    virtual bool init();  
    
    // a selector callback
    void menuCloseCallback(cocos2d::Ref* pSender);

	void menuChangeTouchModeCallback(cocos2d::Ref* pSender);

	void menuSetMoveTypeCallback(Ref* pSender, Widget::TouchEventType type);

	void menuSetAddTypeCallback(Ref* pSender, Widget::TouchEventType type);

	void menuSetAddCustomTypeCallback(Ref* pSender, Widget::TouchEventType type);

	void menuDeleteCallback(Ref* pSender, Widget::TouchEventType type);

	void menuConfirmCallback(Ref* pSender, Widget::TouchEventType type);

    // implement the "static create()" method manually
    CREATE_FUNC(HelloWorld);

	void draw(Renderer *renderer, const Mat4 &transform, uint32_t flags);
	void onDraw(const Mat4 &transform, uint32_t flags);
	void update(float dt);

	bool onTouchBegan(Touch* touch, Event* event);
	void onTouchEnded(Touch* touch, Event* event);
	void onTouchMoved(Touch* touch, Event* event);

	void addUI();

	void selectedItemEvent(Ref *pSender, ListView::EventType type);

	void onExit();

	void addMark(const Vec2& pos);

	DrawNode* getMark(const Vec2& pos);

	void addCustomPolygon();

private:
		CustomCommand _customCmd;
		LabelTTF* _label;
		Vector<DrawNode*> _marks;
		DrawNode* _movingMark;
		bool _isDelete;
};

#endif // __HELLOWORLD_SCENE_H__
