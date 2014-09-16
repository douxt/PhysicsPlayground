#include "MenuLayer.h"
#include "MainScene.h"

bool MenuLayer::init()
{
	if(!Layer::init())
		return false;

	addUI();
	return true;
}


void MenuLayer::addUI()
{
	Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();
	_popCurrent = nullptr;

	_label = LabelTTF::create("Hello World", "Arial", 24);
    _label->setString("   Mode:Move");
	_label->setPosition(origin.x + _label->getContentSize().width, origin.y + visibleSize.height - _label->getContentSize().height);
	this->addChild(_label);
//	addUI();
	auto pop = PopMenu::create();
	pop->addButton("Move",[](){log("Test1 Touched!");});
	pop->addButton("Add Regular",[](){log("Test2 Touched!");});
	pop->addButton("Add Custom",[](){log("Test3 Touched!");});
	pop->addButton("Add Joint",[](){log("Test4 Touched!");});
	pop->setPosition(origin.x + visibleSize.width - pop->getListViewContentSize().width, origin.y + visibleSize.height);
	this->addChild(pop);
	pop->setLocalZOrder(100);
	pop->popEnter();

	
	pop->setCallback("Move",[&](){
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::MOVE_TYPE);
		_label->setString("Mode:Move");
		if(_popCurrent)
			_popCurrent->popExit();
		_popCurrent = nullptr;
	});

	PopMenu* popRegular = PopMenu::create();	
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
	popRegular->setMargin(10);
	this->addChild(popRegular);

	pop->setCallback("Add Regular",[&,popRegular](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::ADD_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_TYPE);
			_label->setString("Mode:Add Regular");
			popRegular->popEnter();
			if(_popCurrent)
				_popCurrent->popExit();
			_popCurrent = popRegular;
		}
	});

	PopMenu* popCustom = PopMenu::create();
	popCustom->addButton("Add",[&](){_main->setIsDelete(false);});
	popCustom->addButton("Delete",[&](){_main->setIsDelete(true);});
	popCustom->addButton("Clear",[&](){_main->clearMarks();});
	popCustom->addButton("Confirm",[&](){_main->addCustomPolygon();});
	popCustom->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popCustom->setMargin(10);
	this->addChild(popCustom);

	pop->setCallback("Add Custom",[&,popCustom](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::ADD_CUSTOM_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_CUSTOM_TYPE);
			_label->setString("Mode:Add Custom");
			_main->setIsDelete(false);
			popCustom->popEnter();
			if(_popCurrent)
				_popCurrent->popExit();
			_popCurrent = popCustom;
		}
	});

	PopMenu* popJoint = PopMenu::create();
	popJoint->addButton("Mark",[](){});
	popJoint->addButton("Create",[](){});
	popJoint->addButton("Add Mark",[](){});
	popJoint->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popJoint->setMargin(10);
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
		}
	});
}

void MenuLayer::setMainScene(MainScene* mainScene)
{
	_main = mainScene;
}