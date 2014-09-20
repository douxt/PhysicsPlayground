#include "PhysicsManager.h"

PhysicsManager* PhysicsManager::_physicsManager = nullptr;

PhysicsManager::PhysicsManager():_world(nullptr),_debugDraw(nullptr),_sideNum(0),
_touchType(TouchType::MOVE_TYPE),_mouseWorld(b2Vec2(0, 0)),_mouseJoint(nullptr),
_groundBody(nullptr),_car(nullptr),_wheel(nullptr),_movingBody(nullptr),
_jointType(b2JointType::e_unknownJoint),_collideConnected(false)
{}

PhysicsManager::~PhysicsManager()
{
	delete _world;
	_world = nullptr;
	delete _debugDraw;
	_debugDraw = nullptr;
}

PhysicsManager* PhysicsManager::getInstance()
{
	if(_physicsManager)
		return _physicsManager;
	_physicsManager = new PhysicsManager();
	if(_physicsManager && _physicsManager->init())
	{
		_physicsManager->autorelease();
		CC_SAFE_RETAIN(_physicsManager);
		return _physicsManager;
	}
	else
	{
		CC_SAFE_DELETE(_physicsManager);
		return nullptr;
	}
}

void PhysicsManager::purgeInstance()
{
	CC_SAFE_RELEASE_NULL(_physicsManager);
}

bool PhysicsManager::init()
{
	Size sz = Director::getInstance()->getWinSize();
	b2Vec2 gravity;
	gravity.Set(0, -8);
	_world = new b2World(gravity);
	_world->SetAllowSleeping(true);
	_world->SetContinuousPhysics(true);

//	std::vector<int> vi = {1,2,3};
	_debugDraw = new GLESDebugDraw(PTM_RATIO);
	_world->SetDebugDraw(_debugDraw);
	_isMotorEnabled = true;
	uint32 flags = 0;
	flags += b2Draw::e_shapeBit
			+b2Draw::e_jointBit
//			+b2Draw::e_aabbBit
			+b2Draw::e_pairBit
//			+b2Draw::e_centerOfMassBit
			;
	_debugDraw->SetFlags(flags);

	b2BodyDef bodyDef;
	_groundBody = _world->CreateBody(&bodyDef);
	_size = 50;
	_isPaused = false;
	_fixtureDef.density = 15;
	_fixtureDef.friction = 0.4f;
	_fixtureDef.restitution = 0.6f;

	_motorSpeed = 10.0f;
	_maxMotorTorque = 800.0f;
//	_wheelJointDef.enableMotor = true;
	_frequencyHz = 4.0f;
	_dampingRatio = 0.7f;
//	addBlock(Point(500, 500),Size(100, 50));
	addRegularPolygon(Point(500, 500));


	b2BodyDef tableBodyDef;
	tableBodyDef.position.Set(0, 0);
	b2Body* tableBody = _world->CreateBody(&tableBodyDef);

	b2EdgeShape tableBox;
	
	//bottom
	tableBox.Set(b2Vec2(-sz.width/PTM_RATIO, 0.0f), b2Vec2(sz.width/PTM_RATIO, 0.0f));
	tableBody->CreateFixture(&tableBox, 0);

	//top
	tableBox.Set(b2Vec2(-sz.width/PTM_RATIO, 2*sz.height/PTM_RATIO), b2Vec2(sz.width/PTM_RATIO, 2*sz.height/PTM_RATIO));
	tableBody->CreateFixture(&tableBox, 0);

	//left
	tableBox.Set(b2Vec2(-sz.width/PTM_RATIO, 0.0f), b2Vec2(-sz.width/PTM_RATIO, 2*sz.height/PTM_RATIO));
	tableBody->CreateFixture(&tableBox, 0);

	//right
	tableBox.Set(b2Vec2(sz.width/PTM_RATIO, 0.0f), b2Vec2(sz.width/PTM_RATIO, 2*sz.height/PTM_RATIO));
	tableBody->CreateFixture(&tableBox, 0);

	return true;
}

void PhysicsManager::addRegularPolygon(Point pos)
{
	if(_sideNum == 0)
	{
		addCircle(pos, _size);
		return;
	}
	b2BodyDef bodydef;
	bodydef.type = b2_dynamicBody;

	auto body = _world->CreateBody(&bodydef);
	body->SetSleepingAllowed(true);
	body->SetLinearDamping(0.2);
	body->SetAngularDamping(0.2);

	int num = _sideNum;
	auto radias =  _size/PTM_RATIO;

	auto points = new b2Vec2[num];
	
	for(int i=0; i<num; i++)
	{
		float angle = (float)i/num*2*b2_pi;
		points[i] = b2Vec2(radias*cos(angle), radias*sin(angle));
	}
//	_body->SetFixedRotation(true);
	b2PolygonShape shape;
//	shape.SetAsBox(size.width/PTM_RATIO/2, size.height/PTM_RATIO/2);//, b2Vec2(-sz.width/PTM_RATIO/2, -sz.height/PTM_RATIO/2), 0
//	b2Vec2 points[] ={b2Vec2(-1,0), b2Vec2(1, 0), b2Vec2(0, sqrt(3))};
	shape.Set(points, num);

	b2FixtureDef fixtureDef(_fixtureDef);
	fixtureDef.shape = &shape;
	body->CreateFixture(&fixtureDef);

	if(body){
		body->SetTransform(b2Vec2(
			pos.x / PTM_RATIO,
			pos.y / PTM_RATIO),
			body->GetAngle());
	}
	delete [] points;
}

void PhysicsManager::addCircle(Point pos, float radias)
{
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;

	auto body = _world->CreateBody(&bodyDef);
	body->SetSleepingAllowed(true);
	body->SetLinearDamping(0.2);
	body->SetAngularDamping(0.2);

	b2CircleShape shape;
	shape.m_radius = radias / PTM_RATIO;

	b2FixtureDef fixtureDef(_fixtureDef);
	fixtureDef.shape = &shape;
	body->CreateFixture(&fixtureDef);

	if(body){
		body->SetTransform(b2Vec2(
			pos.x / PTM_RATIO,
			pos.y / PTM_RATIO),
			body->GetAngle());
	}
}

void PhysicsManager::addCustomPolygon(const std::vector<Vec2>& points)
{
	b2BodyDef bodydef;
	bodydef.type = b2_dynamicBody;

	auto body = _world->CreateBody(&bodydef);
	body->SetSleepingAllowed(true);
	body->SetLinearDamping(0.2);
	body->SetAngularDamping(0.2);

	int num = points.size();

	auto pts = new b2Vec2[num];
	b2Vec2 middle = b2Vec2(0, 0);

	for(int i=0; i<num; i++)
	{
		middle += b2Vec2(points[i].x / PTM_RATIO, points[i].y / PTM_RATIO);
	}
	middle = b2Vec2(middle.x/num, middle.y/num);

	for(int i=0; i<num; i++)
	{
		pts[i] = b2Vec2(points[i].x / PTM_RATIO - middle.x, points[i].y / PTM_RATIO - middle.y);
	}
//	_body->SetFixedRotation(true);
	b2PolygonShape shape;
//	shape.SetAsBox(size.width/PTM_RATIO/2, size.height/PTM_RATIO/2);//, b2Vec2(-sz.width/PTM_RATIO/2, -sz.height/PTM_RATIO/2), 0
//	b2Vec2 points[] ={b2Vec2(-1,0), b2Vec2(1, 0), b2Vec2(0, sqrt(3))};
	shape.Set(pts, num);

	b2FixtureDef fixtureDef(_fixtureDef);
	fixtureDef.shape = &shape;
	body->CreateFixture(&fixtureDef);

	if(body){
		body->SetTransform(
			middle,
			body->GetAngle());
	}
	delete [] pts;
}

bool PhysicsManager::isMovingBody()
{
	return (_touchType == MOVE_TYPE) && (_mouseJoint != nullptr || _movingBody != nullptr);
}


class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		m_point = point;
		m_fixture = nullptr;
	}

	bool ReportFixture(b2Fixture* fixture)
	{
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody)
		{
			bool inside = fixture->TestPoint(m_point);
			if (inside)
			{
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};


bool PhysicsManager::MouseDown(const Vec2& pos)
{
	b2Vec2 p = b2Vec2(pos.x/PTM_RATIO, pos.y/PTM_RATIO);
	_mouseWorld = p;

	if(_isPaused)
	{
		return pauseMouseDown(pos);
	}

	if (_mouseJoint != nullptr)
	{
		return false;
	}

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	_world->QueryAABB(&callback, aabb);

	if (callback.m_fixture)
	{
		b2Body* body = callback.m_fixture->GetBody();
		b2MouseJointDef md;
		md.bodyA = _groundBody;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 1000.0f * body->GetMass();
		_mouseJoint = (b2MouseJoint*)_world->CreateJoint(&md);
		body->SetAwake(true);
        return true;
	}
    
    return true;
}

bool PhysicsManager::pauseMouseDown(const Vec2& pos)
{
	if(_movingBody)
		return false;

	b2Body* body = getBodyAt(pos);
	if(body)
	{
		_movingBody = body;
		return true;
	}
	return true;
}

void PhysicsManager::MouseUp(const Vec2& pos)
{
	if (_mouseJoint)
	{
		_world->DestroyJoint(_mouseJoint);
		_mouseJoint = nullptr;
	}
	if(_movingBody)
	{
		_movingBody = nullptr;
	}
}

void PhysicsManager::MouseMove(const Vec2& pos)
{
	b2Vec2 p = b2Vec2(pos.x/PTM_RATIO, pos.y/PTM_RATIO);
	b2Vec2 delta = p - _mouseWorld;
	_mouseWorld = p;
	
	if (_mouseJoint)
	{
		_mouseJoint->SetTarget(p);
	}

	if(_movingBody)
	{
		_movingBody->SetTransform(_movingBody->GetPosition() + delta, _movingBody->GetAngle());
	}
}

b2Body* PhysicsManager::getBodyAt(const Vec2& pos)
{
	b2Vec2 p = b2Vec2(pos.x/PTM_RATIO, pos.y/PTM_RATIO);
	
	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = p - d;
	aabb.upperBound = p + d;

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	_world->QueryAABB(&callback, aabb);

	if (callback.m_fixture)
	{
		b2Body* body = callback.m_fixture->GetBody();
		if(body)
			return body;
	}
    
    return nullptr;
}

void PhysicsManager::deleteBodyAt(const Vec2& pos)
{
	b2Vec2 p = b2Vec2(pos.x/PTM_RATIO, pos.y/PTM_RATIO);
	b2Body* body = getBodyAt(pos);
	if(body)
	{
		_world->DestroyBody(body);
	}
}

void PhysicsManager::addWheelJoint(const Vec2& pos)
{
	b2Vec2 p = b2Vec2(pos.x/PTM_RATIO, pos.y/PTM_RATIO);
	b2Body* body = getBodyAt(pos);
	if(body)
	{
		if(!_car)
		{
			_car = body;
		}
		else if(_car != body)
		{
			auto m_hz = 4.0f;
			auto m_zeta = 0.7f;
			_wheel = body;
			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);
			jd.Initialize(_car, _wheel, _wheel->GetPosition(), axis);
			jd.enableMotor = _isMotorEnabled;
			jd.motorSpeed = _motorSpeed;
			jd.maxMotorTorque = _maxMotorTorque;
			jd.frequencyHz = _frequencyHz;
			jd.dampingRatio = _dampingRatio;
			jd.collideConnected = _collideConnected;
			_world->CreateJoint(&jd);
			_car = nullptr;
			_wheel = nullptr;
		}
	}
}

void PhysicsManager::addJoint(const Vec2& pos1, const Vec2& pos2)
{
	auto body1 = getBodyAt(pos1);
	auto body2 = getBodyAt(pos2);
	if(body1 && body2)
	{
		auto p1 = b2Vec2(pos1.x/PTM_RATIO, pos1.y/PTM_RATIO);
		auto p2 = b2Vec2(pos2.x/PTM_RATIO, pos2.y/PTM_RATIO);

		b2DistanceJointDef djd;
		djd.dampingRatio = _dampingRatio;
		djd.frequencyHz = _frequencyHz;
		djd.collideConnected = _collideConnected;
		djd.Initialize(body1, body2, p1, p2);
		_world->CreateJoint(&djd);
	}
}

void PhysicsManager::pause()
{
	_isPaused = true;
}

void PhysicsManager::resume()
{
	_isPaused = false;
}

void PhysicsManager::update(float dt)
{
	if(!_isPaused)
	{
		_world->Step(dt, 8, 8);
	}
}

void PhysicsManager::togglePause()
{
	_isPaused = !_isPaused;
}

void PhysicsManager::setGravity(const Vec2& gravity)
{
	_world->SetGravity(b2Vec2(gravity.x, gravity.y));
}
bool PhysicsManager::getPropertyByNameBool(const std::string &name)
{
	if("EnableMotor" == name)
	{
		return _isMotorEnabled;
	}
	if("CollideConnected" == name)
	{
		return _collideConnected;
	}
	log("No such property as: %s, return NULL", name.c_str());
	return NULL;	
}

void PhysicsManager::setPropertyByNameBool(const std::string &name, bool bval)
{
	if("EnableMotor" == name)
	{
		_isMotorEnabled = bval;
	}

	if("CollideConnected" == name)
	{
		_collideConnected = bval;
	}
	log("No such property as: %s, nothing set", name.c_str());
}

float PhysicsManager::getPropertyByName(const std::string &name)
{
	if("Size" == name)
	{
		return _size;
	}
	if("Density" == name)
	{
		return _fixtureDef.density;
	}
	if("Friction" == name)
	{
		return _fixtureDef.friction;
	}
	if("Restitution" == name)
	{
		return _fixtureDef.restitution;
	}
	if("MotorSpeed" == name)
	{

			return _motorSpeed;

	}
	if("MaxMotorTorque" == name)
	{

			return _maxMotorTorque;

	}
	if("FrequencyHz" == name)
	{

			return _frequencyHz;

	}
	if("DampingRatio" == name)
	{

			return _dampingRatio;

	}
	log("No such property as: %s, return NULL", name.c_str());
	return NULL;
}

Vec2 PhysicsManager::getRangeByName(const std::string &name)
{
	if("Size" == name)
	{
		return Vec2(1, 500);
	}
	if("Density" == name)
	{
		return Vec2(0, 500);
	}
	if("Friction" == name)
	{
		return Vec2(0, 5);
	}
	if("Restitution" == name)
	{
		return Vec2(0, 2);
	}
	if("MotorSpeed" == name)
	{
		return Vec2(-100, 100);
	}

	if("MaxMotorTorque" == name)
	{
			return Vec2(0, 1000);
	}
	if("FrequencyHz" == name)
	{
			return Vec2(0, 30);
	}
	if("DampingRatio" == name)
	{
			return Vec2(0, 5);
	}
	log("No such property as: %s, range return NULL", name.c_str());
	return NULL;
}
void PhysicsManager::setPropertyByName(const std::string& name, float fval)
{
	if("Size" == name)
	{
		_size =  fval;
		return;
	}
	if("Density" == name)
	{
		_fixtureDef.density =  fval;
		return;
	}
	if("Friction" == name)
	{
		_fixtureDef.friction =  fval;
		return;
	}
	if("Restitution" == name)
	{
		_fixtureDef.restitution =  fval;
		return;
	}

	if("MotorSpeed" == name)
	{
			_motorSpeed = fval;
			return;
	}

	if("MaxMotorTorque" == name)
	{

			_maxMotorTorque = fval;

	}
	if("FrequencyHz" == name)
	{

			_frequencyHz = fval;

	}
	if("DampingRatio" == name)
	{

			_dampingRatio = fval;

	}
	log("No such property as: %s, nothing set", name.c_str());
}