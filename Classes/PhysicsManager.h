#ifndef __PhysicsManager__
#define __PhysicsManager__
#include "cocos2d.h"
#include "Box2D\Box2D.h"
#include "GLES-Render.h"

USING_NS_CC;

#define PTM_RATIO 32.0
class PhysicsManager : public Ref
{
public:

	enum TouchType
	{
		MOVE_TYPE,
		ADD_TYPE,
		ADD_CUSTOM_TYPE,
		ADD_JOINT_TYPE		
	};
	static PhysicsManager* getInstance();
	static void purgeInstance();
	bool init();
	void addRegularPolygon(Point pos, float radias);
	bool isMovingBody();
	bool MouseDown(const Vec2& pos);
	void MouseUp(const Vec2& pos);
	void MouseMove(const Vec2& pos);

	void addCustomPolygon(const std::vector<Vec2>& points);
	void addCircle(Point pos, float radias);

	b2Body* getBodyAt(const Vec2& pos);
	void addWheelJoint(const Vec2& pos);

public:
	CC_SYNTHESIZE(TouchType, _touchType, TouchType);
	CC_SYNTHESIZE(b2World*, _world, World);
	CC_SYNTHESIZE(int, _sideNum, SideNum);

private:
	static PhysicsManager* _physicsManager;
	PhysicsManager();
	~PhysicsManager();
	PhysicsManager(const PhysicsManager& other);
	PhysicsManager& operator=(const PhysicsManager& other);

	GLESDebugDraw* _debugDraw;

//	TouchType _touchType;
	b2Vec2 _mouseWorld;
	b2MouseJoint* _mouseJoint;
	b2Body* _groundBody;
	b2Body* _car;
	b2Body* _wheel;
};


#endif