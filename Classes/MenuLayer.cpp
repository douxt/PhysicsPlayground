#include "MenuLayer.h"
#include "MainScene.h"

bool MenuLayer::init()
{
	if(!Layer::init())
		return false;

	addUI();

	NotificationCenter::getInstance()->addObserver(this, callfuncO_selector(MenuLayer::toggleMenu), "toggleMenu", nullptr);

	return true;
}


void MenuLayer::addUI()
{
	Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();
	_popCurrent = nullptr;
	_popCurrent2 = nullptr;
	int margin = 10;
	_label = LabelTTF::create("Hello World", "fonts/Marker Felt.ttf", 24);
    _label->setString("   Mode:Move");
	_label->setPosition(origin.x + _label->getContentSize().width, origin.y + visibleSize.height - _label->getContentSize().height);
	this->addChild(_label);
//	_label->setVisible(false);
//	addUI();

	PopMenu* popAddParameter = PopMenu::create();

	popAddParameter->addSlider("Size");
	popAddParameter->addSlider("Density");
	popAddParameter->addSlider("Friction");
	popAddParameter->addSlider("Restitution");
	popAddParameter->addSlider("BodyType");
	popAddParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popAddParameter);

	PopMenu* popWheelJointParameter = PopMenu::create();
	popWheelJointParameter->addSlider("MotorSpeed");
	popWheelJointParameter->addSlider("MaxMotorTorque");
	popWheelJointParameter->addSlider("FrequencyHz");
	popWheelJointParameter->addSlider("DampingRatio");
	popWheelJointParameter->addCheckBox("EnableMotor");
	popWheelJointParameter->addCheckBox("CollideConnected");
	popWheelJointParameter->addCheckBox("ToGround");
	popWheelJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popWheelJointParameter);
//	popWheelJointParameter->popEnter();

	PopMenu* popDistanceJointParameter = PopMenu::create();
	popDistanceJointParameter->addSlider("FrequencyHz");
	popDistanceJointParameter->addSlider("DampingRatio");
	popDistanceJointParameter->addCheckBox("CollideConnected");
	popDistanceJointParameter->addCheckBox("ToGround");
	popDistanceJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popDistanceJointParameter);


	PopMenu* popRevoluteJointParameter = PopMenu::create();
	popRevoluteJointParameter->addSlider("LowerAngle");
	popRevoluteJointParameter->addSlider("UpperAngle");
	popRevoluteJointParameter->addCheckBox("EnableLimit");
	popRevoluteJointParameter->addSlider("MotorSpeed");
	popRevoluteJointParameter->addSlider("MaxMotorTorque");
	popRevoluteJointParameter->addCheckBox("EnableMotor");
	popRevoluteJointParameter->addCheckBox("CollideConnected");
	popRevoluteJointParameter->addCheckBox("ToGround");
	popRevoluteJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popRevoluteJointParameter);

	PopMenu* popPrismaticJointParameter = PopMenu::create();
	popPrismaticJointParameter->addSlider("LowerTranslation");
	popPrismaticJointParameter->addSlider("UpperTranslation");
	popPrismaticJointParameter->addCheckBox("EnableLimit");
	popPrismaticJointParameter->addSlider("MotorSpeed");
	popPrismaticJointParameter->addSlider("MaxMotorForce");
	popPrismaticJointParameter->addCheckBox("EnableMotor");
	popPrismaticJointParameter->addCheckBox("CollideConnected");
	popPrismaticJointParameter->addCheckBox("ToGround");
	popPrismaticJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popPrismaticJointParameter);

	PopMenu* popPulleyJointParameter = PopMenu::create();
	popPulleyJointParameter->addSlider("PulleyRatio");
	popPulleyJointParameter->addCheckBox("CollideConnected");
	popPulleyJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popPulleyJointParameter);

	PopMenu* popWeldJointParameter = PopMenu::create();
	popWeldJointParameter->addSlider("FrequencyHz");
	popWeldJointParameter->addSlider("DampingRatio");
	popWeldJointParameter->addCheckBox("CollideConnected");
	popWeldJointParameter->addCheckBox("ToGround");
	popWeldJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popWeldJointParameter);

	PopMenu* popRopeJointParameter = PopMenu::create();
//	popRopeJointParameter->addSlider("MaxLength");
	popRopeJointParameter->addCheckBox("CollideConnected");
	popRopeJointParameter->addCheckBox("ToGround");
	popRopeJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popRopeJointParameter);

	PopMenu* popFrictionJointParameter = PopMenu::create();
	popFrictionJointParameter->addSlider("MaxForce");
	popFrictionJointParameter->addSlider("MaxTorque");
	popFrictionJointParameter->addCheckBox("CollideConnected");
	popFrictionJointParameter->addCheckBox("ToGround");
	popFrictionJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popFrictionJointParameter);

	PopMenu* popMotorJointParameter = PopMenu::create();
	popMotorJointParameter->addSlider("MaxForce");
	popMotorJointParameter->addSlider("MaxTorque");
	popMotorJointParameter->addCheckBox("CollideConnected");
	popMotorJointParameter->addCheckBox("ToGround");
	popMotorJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popMotorJointParameter);

	PopMenu* popGearJointParameter = PopMenu::create();
	popGearJointParameter->addSlider("GearRatio");
	popGearJointParameter->setPosition(origin.x + visibleSize.width/2, origin.y + visibleSize.height);
	this->addChild(popGearJointParameter);

	auto pop = PopMenu::create();
	pop->addButton("Move",[](){log("Test1 Touched!");});
	pop->addButton("Add Regular",[](){log("Test2 Touched!");});
	pop->addButton("Add Custom",[](){log("Test3 Touched!");});
	pop->addButton("Add Joint",[](){log("Test4 Touched!");});
	pop->addButton("No Collide",[](){log("Test4 Touched!");});
	pop->addButton("Gadgets",[](){log("Test4 Touched!");});
	pop->setPosition(origin.x + visibleSize.width - pop->getListViewContentSize().width, origin.y + visibleSize.height);
	pop->setName("popMain");
	this->addChild(pop);
	pop->setLocalZOrder(100);
	pop->popEnter();

	pop->setCallback("Move",[&,popAddParameter](){
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::MOVE_TYPE);
		_label->setString("Mode:Move");
		if(_popCurrent)
			_popCurrent->popExit();
		_popCurrent = nullptr;
		if(_popCurrent2)
			_popCurrent2->popExit();
		_popCurrent2 = nullptr;
	});

	int maxHeight = (int)(visibleSize.height - pop->getListViewContentSize().height - margin);


	PopMenu* popRegular = PopMenu::create();
	popRegular->setMaxHeight(maxHeight);
	popRegular->addButton("Circle",[](){PhysicsManager::getInstance()->setSideNum(0);});
	popRegular->addButton("Triangle",[](){PhysicsManager::getInstance()->setSideNum(3);});
	popRegular->addButton("Square",[](){PhysicsManager::getInstance()->setSideNum(4);});
	popRegular->addButton("Pentagon",[](){PhysicsManager::getInstance()->setSideNum(5);});
	popRegular->addButton("Hexgon",[](){PhysicsManager::getInstance()->setSideNum(6);});
	popRegular->addButton("Heptagon",[](){PhysicsManager::getInstance()->setSideNum(7);});
	popRegular->addButton("Octagon",[](){PhysicsManager::getInstance()->setSideNum(8);});
	popRegular->addButton("Enneagon",[](){PhysicsManager::getInstance()->setSideNum(9);});
	popRegular->addButton("Decagon",[](){PhysicsManager::getInstance()->setSideNum(10);});	
	popRegular->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popRegular->setMargin(margin);

	this->addChild(popRegular);
//	popRegular->setLocalZOrder(100);
	pop->setCallback("Add Regular",[&,popRegular,popAddParameter](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::ADD_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_TYPE);
			_label->setString("Mode:Add Regular");
			popRegular->popEnter();
			if(_popCurrent)
				_popCurrent->popExit();
			_popCurrent = popRegular;

			popAddParameter->popEnter();
			if(_popCurrent2&&_popCurrent2!=popAddParameter)
				_popCurrent2->popExit();
			_popCurrent2 = popAddParameter;
		}
	});

	PopMenu* popCustom = PopMenu::create();
	popCustom->setMaxHeight(maxHeight);
	popCustom->addButton("Add",[&](){_main->setIsDelete(false);});
	popCustom->addButton("Delete",[&](){_main->setIsDelete(true);});
	popCustom->addButton("Clear",[&](){_main->clearMarks();});
	popCustom->addButton("Confirm",[&](){_main->addCustomPolygon();});
	popCustom->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popCustom->setMargin(margin);
	this->addChild(popCustom);

	pop->setCallback("Add Custom",[&,popCustom,popAddParameter](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::ADD_CUSTOM_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_CUSTOM_TYPE);
			_label->setString("Mode:Add Custom");
			_main->setIsDelete(false);
			popCustom->popEnter();
			if(_popCurrent)
				_popCurrent->popExit();
			_popCurrent = popCustom;

			popAddParameter->popEnter();
			if(_popCurrent2&&_popCurrent2!=popAddParameter)
				_popCurrent2->popExit();
			_popCurrent2 = popAddParameter;

			_main->clearMarks();
			_main->setMaxMark(32);
		}
	});

	PopMenu* popJoint = PopMenu::create();
	popJoint->setMaxHeight(maxHeight);
	popJoint->addButton("Wheel",[&,popWheelJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_wheelJoint);
		_label->setString("Mode:Add Joint Wheel");
		popWheelJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popWheelJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popWheelJointParameter;

		_main->clearMarks();
		_main->setMaxMark(2);
	});
	popJoint->addButton("Distance",[&,popDistanceJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_distanceJoint);
		_label->setString("Mode:Add Joint Distance");
		popDistanceJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popDistanceJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popDistanceJointParameter;

		_main->clearMarks();
		_main->setMaxMark(2);
	});

	popJoint->addButton("Revolute",[&,popRevoluteJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_revoluteJoint);
		_label->setString("Mode:Add Joint Revolute");
		popRevoluteJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popRevoluteJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popRevoluteJointParameter;

		_main->clearMarks();
		_main->setMaxMark(3);
	});

	popJoint->addButton("Prismatic",[&,popPrismaticJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_prismaticJoint);
		_label->setString("Mode:Add Joint Prismatic");
		popPrismaticJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popPrismaticJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popPrismaticJointParameter;

		_main->clearMarks();
		_main->setMaxMark(4);
	});

	popJoint->addButton("Pulley",[&,popPulleyJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_pulleyJoint);
		_label->setString("Mode:Add Joint Pulley");
		popPulleyJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popPulleyJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popPulleyJointParameter;

		_main->clearMarks();
		_main->setMaxMark(4);
	});

	popJoint->addButton("Weld",[&,popWeldJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_weldJoint);
		_label->setString("Mode:Add Joint Weld");
		popWeldJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popWeldJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popWeldJointParameter;

		_main->clearMarks();
		_main->setMaxMark(3);
	});

	popJoint->addButton("Rope",[&,popRopeJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_ropeJoint);
		_label->setString("Mode:Add Joint Rope");
		popRopeJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popRopeJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popRopeJointParameter;

		_main->clearMarks();
		_main->setMaxMark(2);
	});
	popJoint->addButton("Friction",[&,popFrictionJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_frictionJoint);
		_label->setString("Mode:Add Joint Friction");
		popFrictionJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popFrictionJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popFrictionJointParameter;

		_main->clearMarks();
		_main->setMaxMark(2);
	});
	popJoint->addButton("Motor",[&,popMotorJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_motorJoint);
		_label->setString("Mode:Add Joint Motor");
		popMotorJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popMotorJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popMotorJointParameter;

		_main->clearMarks();
		_main->setMaxMark(2);
	});
	popJoint->addButton("Gear",[&,popGearJointParameter](){
		PhysicsManager::getInstance()->setJointType(b2JointType::e_gearJoint);
		_label->setString("Mode:Add Joint Gear");
		popGearJointParameter->popEnter();
		if(_popCurrent2&&_popCurrent2!=popGearJointParameter)
			_popCurrent2->popExit();
		_popCurrent2 = popGearJointParameter;

		_main->clearMarks();
		_main->setMaxMark(2);
	});
	popJoint->addButton("Create",[&](){_main->addJoint();});
	popJoint->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popJoint->setMargin(margin);

	this->addChild(popJoint);

	pop->setCallback("Add Joint",[&,popJoint](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::ADD_JOINT_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_JOINT_TYPE);
			_label->setString("Mode:Add Joint");
			popJoint->popEnter();
			if(_popCurrent)
				_popCurrent->popExit();
			_popCurrent = popJoint;
			if(_popCurrent2&&_popCurrent2!=nullptr)
				_popCurrent2->popExit();
			_popCurrent2 = nullptr;

			PhysicsManager::getInstance()->setJointType(b2JointType::e_unknownJoint);
		}
	});

	PopMenu* popNoCollide = PopMenu::create();
	popNoCollide->setMaxHeight(maxHeight);
	popNoCollide->addButton("Collide",[&](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::COLLIDE_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::COLLIDE_TYPE);
			_label->setString("Mode:Collide");

//			PhysicsManager::getInstance()->setJointType(b2JointType::e_unknownJoint);
		}
	});
	popNoCollide->addButton("Confirm",[&](){
//		PhysicsManager::getInstance()->setJointType(b2JointType::e_wheelJoint);
//		_label->setString("Mode:Add Joint Wheel");
		_main->addNoCollide();
	});
	popNoCollide->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popNoCollide->setMargin(margin);
	this->addChild(popNoCollide);

	pop->setCallback("No Collide",[&,popNoCollide](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::NO_COLLIDE_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::NO_COLLIDE_TYPE);
			_label->setString("Mode:No Collide");
			popNoCollide->popEnter();
			if(_popCurrent&&_popCurrent!=popNoCollide)
				_popCurrent->popExit();
			_popCurrent = popNoCollide;
			if(_popCurrent2&&_popCurrent2!=nullptr)
				_popCurrent2->popExit();
			_popCurrent2 = nullptr;

			_main->clearMarks();
			_main->setMaxMark(2);
//			PhysicsManager::getInstance()->setJointType(b2JointType::e_unknownJoint);
		}
	});

	PopMenu* popGadgets = PopMenu::create();
	popGadgets->setMaxHeight(maxHeight);
	popGadgets->addButton("Thruster",[&](){
		PhysicsManager::getInstance()->setGadgetType(PhysicsManager::GADGET_THRUSTER);
		_label->setString("Mode:Gadgets-Thruster");
	});
	popGadgets->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popGadgets->setMargin(margin);
	this->addChild(popGadgets);

	pop->setCallback("Gadgets",[&,popGadgets](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::ADD_GADGET)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_GADGET);
			_label->setString("Mode:Gadgets");
			popGadgets->popEnter();
			if(_popCurrent&&_popCurrent!=popGadgets)
				_popCurrent->popExit();
			_popCurrent = popGadgets;
			if(_popCurrent2&&_popCurrent2!=nullptr)
				_popCurrent2->popExit();
			_popCurrent2 = nullptr;
		}
	});



	PopMenu* popPause = PopMenu::create();
	popPause->addButton("Pause", nullptr);
	popPause->addButton("Gravity", nullptr);
	popPause->addButton("Delete", nullptr);
	popPause->addButton("New", nullptr);
	popPause->addButton("Save", nullptr);
	popPause->addButton("Load", nullptr);
	popPause->addButton("Next", nullptr);
	popPause->setPosition(origin.x, origin.y + visibleSize.height);
	popPause->setMargin(50);
	popPause->setName("popPause");
	this->addChild(popPause);

	popPause->setCallback("Pause", [&,popPause](){
		PhysicsManager::getInstance()->pause();
		popPause->reName("Pause","Resume");
	});

	popPause->setCallback("Resume",[&,popPause](){
		PhysicsManager::getInstance()->resume();
		popPause->reName("Resume","Pause");
	});

	popPause->setCallback("Gravity",[&](){
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::SET_GRAVITY_TYPE);
		_label->setString("Mode:Set Gravity");
	});

	popPause->setCallback("Delete",[&](){
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::DELETE_TYPE);
		_label->setString("Mode:Delete body");
	});

	popPause->setCallback("New",[&](){
		PhysicsManager::getInstance()->newSave();
	});

	popPause->setCallback("Save",[&](){
		PhysicsManager::getInstance()->save();
	});

	popPause->setCallback("Load",[&](){
		PhysicsManager::getInstance()->load();
	});

	popPause->setCallback("Next",[&](){
		PhysicsManager::getInstance()->next();
	});

	popPause->popEnter();

	PopMenu* popController = PopMenu::create();
	popController->addSlider("Controller00");
	popController->setPosition(origin.x , origin.y);
	popController->setIsPopDown(false);
	popController->popEnter();
	this->addChild(popController);

}

void MenuLayer::sliderEvent(Ref *pSender, Slider::EventType type)
{
    if (type == Slider::EventType::ON_PERCENTAGE_CHANGED)
    {
        Slider* slider = dynamic_cast<Slider*>(pSender);
        int percent = slider->getPercent();
        _valueLabel->setString(String::createWithFormat("Percent %d", percent)->getCString());
    }
}

void MenuLayer::setMainScene(MainScene* mainScene)
{
	_main = mainScene;
}

void MenuLayer::toggleMenu(Ref* sender)
{
	PopMenu* pop = dynamic_cast<PopMenu*>(this->getChildByName("popMain"));
	pop->popToggle();
	pop = dynamic_cast<PopMenu*>(this->getChildByName("popPause"));
	pop->popToggle();
}