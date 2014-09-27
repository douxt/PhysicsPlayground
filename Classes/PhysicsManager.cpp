#include "PhysicsManager.h"

PhysicsManager* PhysicsManager::_physicsManager = nullptr;

PhysicsManager::PhysicsManager():_world(nullptr),_debugDraw(nullptr),_sideNum(0),
_touchType(TouchType::MOVE_TYPE),_mouseWorld(b2Vec2(0, 0)),_mouseJoint(nullptr),
_groundBody(nullptr),_car(nullptr),_wheel(nullptr),_movingBody(nullptr),
_jointType(b2JointType::e_unknownJoint),_collideConnected(false),_bodyType(0),_toGround(false),
_pulleyRatio(1),_maxLength(0),_maxForce(0),_maxTorque(0),_gearRatio(1)
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
	_enableMotor = true;
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
	b2EdgeShape shape;
	shape.Set(b2Vec2(-40.0f, 0.0f), b2Vec2(40.0f, 0.0f));
	_groundBody ->CreateFixture(&shape, 0.0f);
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

	_lowerAngle = -45;
	_upperAngle = 45;
	_enableLimit = true;
	_lowerTranslation = -2.5f;
	_upperTranslation = 5.0f;
	_maxMotorForce = 10;
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
	int bt = _bodyType;
	bodydef.type = getBodyType(bt);

	auto body = _world->CreateBody(&bodydef);
	body->SetSleepingAllowed(true);
	body->SetLinearDamping(0);
	body->SetAngularDamping(0);

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
	int bt = _bodyType;
//	bodyDef.type = b2_dynamicBody;
	bodyDef.type = getBodyType(bt);
	auto body = _world->CreateBody(&bodyDef);
	body->SetSleepingAllowed(true);
	body->SetLinearDamping(0);
	body->SetAngularDamping(0);

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
//	bodydef.type = b2_dynamicBody;
	int bt = _bodyType;
	bodydef.type = getBodyType(bt);
	auto body = _world->CreateBody(&bodydef);
	body->SetSleepingAllowed(true);
	body->SetLinearDamping(0);
	body->SetAngularDamping(0);

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
//		if (body->GetType() == b2_dynamicBody)
//		if (body->GetType() >= 0)
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
		_bodyDelete.push_back(body);
//		_world->DestroyBody(body);
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
			jd.enableMotor = _enableMotor;
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

void PhysicsManager::addJoint(const Vec2& pos1, const Vec2& pos2, const Vec2& pos3, const Vec2& pos4)
{
	auto body1 = getBodyAt(pos1);
	auto body2 = getBodyAt(pos2);

	if((body1 || _toGround)&& body2)
	{
		auto p1 = b2Vec2(pos1.x/PTM_RATIO, pos1.y/PTM_RATIO);
		auto p2 = b2Vec2(pos2.x/PTM_RATIO, pos2.y/PTM_RATIO);
		auto p3 = b2Vec2(pos3.x/PTM_RATIO, pos3.y/PTM_RATIO);
		auto p4 = b2Vec2(pos4.x/PTM_RATIO, pos4.y/PTM_RATIO);

		if(_toGround)
			body1 = _groundBody;
		if(_jointType == b2JointType::e_wheelJoint)
		{
			b2WheelJointDef wjd;
			b2Vec2 axis(0.0f, 1.0f);
			wjd.Initialize(body1, body2, body2->GetPosition(), axis);
			wjd.enableMotor = _enableMotor;
			wjd.motorSpeed = _motorSpeed;
			wjd.maxMotorTorque = _maxMotorTorque;
			wjd.frequencyHz = _frequencyHz;
			wjd.dampingRatio = _dampingRatio;
			wjd.collideConnected = _collideConnected;
			_world->CreateJoint(&wjd);
		}
		if(_jointType == b2JointType::e_distanceJoint)
		{
			b2DistanceJointDef djd;
			djd.dampingRatio = _dampingRatio;
			djd.frequencyHz = _frequencyHz;
			djd.collideConnected = _collideConnected;
			djd.Initialize(body1, body2, p1, p2);
			_world->CreateJoint(&djd);
		}
		if(_jointType == b2JointType::e_revoluteJoint)
		{
			b2RevoluteJointDef rjd;
			rjd.Initialize(body1, body2, p3);
			rjd.upperAngle = _upperAngle/180*b2_pi;
			rjd.lowerAngle = _lowerAngle/180*b2_pi;
			rjd.enableLimit = _enableLimit;
			rjd.motorSpeed = _motorSpeed;
			rjd.maxMotorTorque = _maxMotorTorque;
			rjd.enableMotor = _enableMotor;
			rjd.collideConnected = _collideConnected;
			_world->CreateJoint(&rjd);
		}
		if(_jointType == b2JointType::e_prismaticJoint)
		{
			b2PrismaticJointDef pjd;
			b2Vec2 axis;
			if(_toGround)
			{
				axis = b2Vec2(0, 1);
			}else
			{
				auto delta = p1 - p2;
				auto nor = delta.Normalize();
				axis = -b2Vec2(delta.x/nor, delta.y/nor);
			}

			pjd.Initialize(body1, body2, p3, axis);
			pjd.upperTranslation = _upperTranslation;
			pjd.lowerTranslation = _lowerTranslation;
			pjd.enableLimit = _enableLimit;
			pjd.motorSpeed = _motorSpeed;
			pjd.maxMotorForce = _maxMotorForce;
			pjd.enableMotor = _enableMotor;
			pjd.collideConnected = _collideConnected;
			_world->CreateJoint(&pjd);
		}
		if(_jointType == b2JointType::e_pulleyJoint)
		{
			b2PulleyJointDef pjd;

			pjd.Initialize(body1, body2, p3, p4, p1, p2, _pulleyRatio);
			pjd.collideConnected = _collideConnected;
			_world->CreateJoint(&pjd);
		}
		if(_jointType == b2JointType::e_weldJoint)
		{
			b2WeldJointDef wjd;
			wjd.dampingRatio = _dampingRatio;
			wjd.frequencyHz = _frequencyHz;
			wjd.collideConnected = _collideConnected;
			wjd.Initialize(body1, body2, p3);
			_world->CreateJoint(&wjd);
		}
		if(_jointType == b2JointType::e_ropeJoint)
		{
			b2RopeJointDef rjd;
//			rjd.maxLength = _maxLength/PTM_RATIO;
			rjd.maxLength = (p2 - p1).Length();
			rjd.bodyA = body1;
			rjd.bodyB = body2;
			rjd.localAnchorA = body1->GetLocalPoint(p1);
			rjd.localAnchorB = body2->GetLocalPoint(p2);
			rjd.collideConnected = _collideConnected;
		
			_world->CreateJoint(&rjd);
		}

		if(_jointType == b2JointType::e_frictionJoint)
		{
			b2FrictionJointDef fjd;
			fjd.localAnchorA.SetZero();
			fjd.localAnchorB.SetZero();
			fjd.bodyA = body1;
			fjd.bodyB = body2;
			fjd.collideConnected = _collideConnected;
			fjd.maxForce = _maxForce;
			fjd.maxTorque = _maxTorque;

			_world->CreateJoint(&fjd);
		}
		if(_jointType == b2JointType::e_motorJoint)
		{
			b2MotorJointDef mjd;
			mjd.Initialize(body1, body2);
			mjd.collideConnected = _collideConnected;
			mjd.maxForce = _maxForce;
			mjd.maxTorque = _maxTorque;

			_world->CreateJoint(&mjd);
		}
		if(_jointType == b2JointType::e_gearJoint)
		{
			auto joint1 = getJointForGear(body1);
			auto joint2 = getJointForGear(body2);
			if(joint1 && joint2)
			{
				b2GearJointDef gjd;
				gjd.bodyA = body1;
				gjd.bodyB = body2;
				gjd.joint1 = joint1;
				gjd.joint2 = joint2;
				gjd.ratio = _gearRatio;
				gjd.collideConnected = _collideConnected;
				_world->CreateJoint(&gjd);
			}else
			{
				log("Create Gear Joint Failed: joint must be revolute or prismatic!");
			}

		}

	}
}

b2Joint* PhysicsManager::getJointForGear(b2Body* body)
{
	for(auto jl = body->GetJointList(); jl; jl = jl->next)
	{
		auto joint = jl->joint;
		auto type = joint->GetType() ;
		if(type == b2JointType::e_revoluteJoint || type == b2JointType::e_prismaticJoint)
		{
			log("find good joint!");
			return joint;
		}		
	}
	return nullptr;
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
	doDelete(); // do not delete while Step running.

}

void PhysicsManager::doDelete()
{
	if(_bodyDelete.size()>0)
	{
		for(auto body : _bodyDelete)
		{
			_jointDelete.clear();
			for(auto jl = body->GetJointList(); jl; jl = jl->next)
			{
				if(jl->joint)
					_jointDelete.push_back(jl->joint);
			}
			for(auto joint : _jointDelete)
			{
				if(joint->GetType() == b2JointType::e_gearJoint)
				{
					_world->DestroyJoint(joint);
				}
			}
			for(auto joint : _jointDelete)
			{
					_world->DestroyJoint(joint);
			}
			_world->DestroyBody(body);
		}
		_bodyDelete.clear();
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
		return _enableMotor;
	}
	if("CollideConnected" == name)
	{
		return _collideConnected;
	}
	if("EnableLimit" == name)
	{
		return _enableLimit;
	}
	if("ToGround" == name)
	{
		return _toGround;
	}
	log("No such property as: %s, return NULL", name.c_str());
	return NULL;	
}

void PhysicsManager::setPropertyByNameBool(const std::string &name, bool bval)
{
	if("EnableMotor" == name)
	{
		_enableMotor = bval;
	}

	if("CollideConnected" == name)
	{
		_collideConnected = bval;
	}
	if("EnableLimit" == name)
	{
		_enableLimit = bval;
	}
	if("ToGround" == name)
	{
		_toGround = bval;
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
	if("LowerAngle" == name)
	{

			return _lowerAngle;

	}
	if("UpperAngle" == name)
	{

			return _upperAngle;

	}
	if("LowerTranslation" == name)
	{

			return _lowerTranslation;

	}
	if("UpperTranslation" == name)
	{

			return _upperTranslation;

	}
	if("MaxMotorForce" == name)
	{

			return _maxMotorForce;

	}
	if("BodyType" == name)
	{

			return _bodyType;

	}
	if("PulleyRatio" == name)
	{

			return _pulleyRatio;

	}
	if("MaxLength" == name)
	{

			return _maxLength;

	}
	if("MaxForce" == name)
	{

			return _maxForce;

	}
	if("MaxTorque" == name)
	{

			return _maxTorque;
	}
	if("GearRatio" == name)
	{

			return _gearRatio;
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
			return Vec2(0, 10000);
	}
	if("FrequencyHz" == name)
	{
			return Vec2(0, 30);
	}
	if("DampingRatio" == name)
	{
			return Vec2(0, 5);
	}
	if("LowerAngle" == name)
	{

			return Vec2(-180, 180);

	}
	if("UpperAngle" == name)
	{

			return Vec2(-180, 180);

	}
	if("LowerTranslation" == name)
	{

			return Vec2(-10, 10);

	}
	if("UpperTranslation" == name)
	{

			return Vec2(-10, 10);

	}
	if("MaxMotorForce" == name)
	{

			return Vec2(0, 1000);

	}
	if("BodyType" == name)
	{

			return Vec2(0, 2);

	}
	if("PulleyRatio" == name)
	{

			return Vec2(0, 10);
	}
	if("MaxLength" == name)
	{

			return Vec2(0, 1000);

	}
	if("MaxForce" == name)
	{

			return Vec2(0, 1000);

	}
	if("MaxTorque" == name)
	{

			return Vec2(0, 1000);

	}
	if("GearRatio" == name)
	{

			return Vec2(0.1, 10.1);
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
	if("LowerAngle" == name)
	{

			_lowerAngle = fval;

	}
	if("UpperAngle" == name)
	{

			_upperAngle = fval;

	}
	if("LowerTranslation" == name)
	{

			_lowerTranslation = fval;

	}
	if("UpperTranslation" == name)
	{

			_upperTranslation = fval;

	}
	if("MaxMotorForce" == name)
	{

			_maxMotorForce = fval;

	}
	if("BodyType" == name)
	{

			_bodyType = fval;

	}
	if("PulleyRatio" == name)
	{

			_pulleyRatio = fval;
	}
	if("MaxLength" == name)
	{

			_maxLength = fval;

	}
	if("MaxForce" == name)
	{

			_maxForce = fval;

	}
	if("MaxTorque" == name)
	{

			_maxTorque = fval;

	}
	if("GearRatio" == name)
	{

			_gearRatio = fval;
	}
	log("No such property as: %s, nothing set", name.c_str());
}

b2BodyType PhysicsManager::getBodyType(int bt)
{
	if(bt == 0)
	{
		return b2_dynamicBody;
	}
	if(bt == 1)
	{
		return  b2_kinematicBody;
	}
	if(bt == 2)
	{
		return  b2_staticBody;
	}
	return b2_dynamicBody;
}