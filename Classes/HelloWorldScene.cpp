#include "HelloWorldScene.h"
#include "PhysicsManager.h"


USING_NS_CC;

const int markRadias = 20;

Scene* HelloWorld::createScene()
{
    // 'scene' is an autorelease object
    auto scene = Scene::create();
    
    // 'layer' is an autorelease object
    auto layer = HelloWorld::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }
    
    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

    /////////////////////////////
    // 2. add a menu item with "X" image, which is clicked to quit the program
    //    you may modify it.

    // add a "close" icon to exit the progress. it's an autorelease object
    auto closeItem = MenuItemImage::create(
                                           "CloseNormal.png",
                                           "CloseSelected.png",
                                           CC_CALLBACK_1(HelloWorld::menuCloseCallback, this));
    
	closeItem->setPosition(Vec2(origin.x + visibleSize.width - closeItem->getContentSize().width/2 ,
                                origin.y + closeItem->getContentSize().height/2));

	auto touchItem = MenuItemImage::create(
                                           "CloseNormal.png",
                                           "CloseSelected.png",
										   CC_CALLBACK_1(HelloWorld::menuChangeTouchModeCallback, this));
    
	touchItem->setPosition(Vec2(origin.x + visibleSize.width - closeItem->getContentSize().width - touchItem->getContentSize().width/2 ,
                                origin.y + touchItem->getContentSize().height/2));

    // create menu, it's an autorelease object
    auto menu = Menu::create(closeItem, touchItem, NULL);
    menu->setPosition(Vec2::ZERO);
    this->addChild(menu, 1);



	this->scheduleUpdate();

	auto listener = EventListenerTouchOneByOne::create();
	listener->onTouchBegan = CC_CALLBACK_2(HelloWorld::onTouchBegan, this);
	listener->onTouchMoved = CC_CALLBACK_2(HelloWorld::onTouchMoved, this);
	listener->onTouchEnded = CC_CALLBACK_2(HelloWorld::onTouchEnded, this);

	_eventDispatcher->addEventListenerWithSceneGraphPriority(listener, this);

	_popCurrent = nullptr;

	_label = LabelTTF::create("Hello World", "Arial", 24);
    _label->setString("Mode: move");
	_label->setPosition(origin.x + _label->getContentSize().width, origin.y + visibleSize.height - _label->getContentSize().height);
	this->addChild(_label);
//	addUI();
	auto pop = PopMenu::create();
	pop->addButton("Move",[](){log("Test1 Touched!");});
	pop->addButton("Add Regular",[](){log("Test2 Touched!");});
	pop->addButton("Add Custom",[](){log("Test3 Touched!");});
	pop->addButton("Add Joint",[](){log("Test4 Touched!");});
	pop->setPosition(origin.x + visibleSize.width - pop->getListViewContentSize().width, origin.y + visibleSize.height);
	this->addChild(pop,100);
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
	popCustom->addButton("Add",[&](){_isDelete = false;});
	popCustom->addButton("Delete",[&](){_isDelete = true;});
	popCustom->addButton("Clear",[&](){
		for(auto mk: _marks)
		{
			mk->removeFromParent();
		}
		_marks.clear();
	});
	popCustom->addButton("Confirm",[&](){addCustomPolygon();});
	popCustom->setPosition(pop->getPosition() - Vec2(0, pop->getListViewContentSize().height));
	popCustom->setMargin(10);
	this->addChild(popCustom);

	pop->setCallback("Add Custom",[&,popCustom](){
		if(PhysicsManager::getInstance()->getTouchType()!=PhysicsManager::ADD_CUSTOM_TYPE)
		{
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_CUSTOM_TYPE);
			_label->setString("Mode:Add Custom");
			_isDelete = false;
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
    return true;
}


void HelloWorld::menuCloseCallback(Ref* pSender)
{
#if (CC_TARGET_PLATFORM == CC_PLATFORM_WP8) || (CC_TARGET_PLATFORM == CC_PLATFORM_WINRT)
	MessageBox("You pressed the close button. Windows Store Apps do not implement a close button.","Alert");
    return;
#endif

    Director::getInstance()->end();

#if (CC_TARGET_PLATFORM == CC_PLATFORM_IOS)
    exit(0);
#endif
}

void HelloWorld::menuChangeTouchModeCallback(Ref* pSender)
{
	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::MOVE_TYPE)
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_TYPE);
	else
			PhysicsManager::getInstance()->setTouchType(PhysicsManager::MOVE_TYPE);
}

void HelloWorld::menuSetMoveTypeCallback(Ref* pSender, Widget::TouchEventType type)
{
	if(type == Widget::TouchEventType::ENDED)
	{
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::MOVE_TYPE);
		_label->setString("Mode:Move");
	}
}

void HelloWorld::menuSetAddTypeCallback(Ref* pSender, Widget::TouchEventType type)
{
	if(type == Widget::TouchEventType::ENDED)
	{
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_TYPE);
		_label->setString("Mode: Add");
	}
}

void HelloWorld::menuSetAddCustomTypeCallback(Ref* pSender, Widget::TouchEventType type)
{
	if(type == Widget::TouchEventType::ENDED)
	{
		PhysicsManager::getInstance()->setTouchType(PhysicsManager::ADD_CUSTOM_TYPE);
		_label->setString("Mode: Add Custom");
		_isDelete = false;
	}
}

void HelloWorld::menuDeleteCallback(Ref* pSender, Widget::TouchEventType type)
{
	if(type == Widget::TouchEventType::ENDED)
	{
		_isDelete = true;
	}
}

void HelloWorld::menuConfirmCallback(Ref* pSender, Widget::TouchEventType type)
{
	if(type == Widget::TouchEventType::ENDED)
	{
		addCustomPolygon();
	}
}


void HelloWorld::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
    Layer::draw(renderer, transform, flags);

    _customCmd.init(_globalZOrder);
    _customCmd.func = CC_CALLBACK_0(HelloWorld::onDraw, this, transform, flags);
    renderer->addCommand(&_customCmd);
}

void HelloWorld::onDraw(const Mat4 &transform, uint32_t flags)
{
    Director* director = Director::getInstance();
    CCASSERT(nullptr != director, "Director is null when seting matrix stack");
    director->pushMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
    director->loadMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW, transform);

    GL::enableVertexAttribs( cocos2d::GL::VERTEX_ATTRIB_FLAG_POSITION );
//    m_test->Step(&settings);
//	_world->DrawDebugData();
	PhysicsManager::getInstance()->getWorld()->DrawDebugData();
    CHECK_GL_ERROR_DEBUG();
    
    director->popMatrix(MATRIX_STACK_TYPE::MATRIX_STACK_MODELVIEW);
}

void HelloWorld::update(float dt)
{
	PhysicsManager::getInstance()->getWorld()->Step(dt, 8, 8);
}


bool HelloWorld::onTouchBegan(Touch* touch, Event* event)
{
	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::ADD_TYPE)
		return true;
	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::ADD_CUSTOM_TYPE)
	{
		auto pos = this->convertToNodeSpace(touch->getLocation());
		_movingMark = getMark(pos);
		if(_movingMark)
		{
			if(_isDelete)
			{
				_marks.eraseObject(_movingMark);
				_movingMark->removeFromParent();
				_movingMark=nullptr;
				return false;
			}
			return true;
		}
		else
		{
			if(!_isDelete)
				addMark(pos);
			return false;
		}
	}
	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::MOVE_TYPE)
	{
		auto touchLocation = touch->getLocation();    
		auto nodePosition = convertToNodeSpace( touchLocation );
		log("HelloWorld::onTouchBegan, pos: %f,%f -> %f,%f", touchLocation.x, touchLocation.y, nodePosition.x, nodePosition.y);
		return PhysicsManager::getInstance()->MouseDown(nodePosition);
//		return MouseDown(b2Vec2(nodePosition.x/PTM_RATIO,nodePosition.y/PTM_RATIO));
	}
	return false;
}

void HelloWorld::onTouchEnded(Touch* touch, Event* event)
{
	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::ADD_TYPE)
	{
		auto pos = this->convertToNodeSpace(touch->getLocation());
//		this->addBlock(pos, Size(100, 50));
		PhysicsManager::getInstance()->addRegularPolygon(pos, 50);
	}

	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::ADD_CUSTOM_TYPE)
	{
		auto pos = this->convertToNodeSpace(touch->getLocation());
		
//		this->addBlock(pos, Size(100, 50));
//		PhysicsManager::getInstance()->addRegularPolygon(pos, 50);
	}

	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::MOVE_TYPE)
	{
		auto touchLocation = touch->getLocation();    
		auto nodePosition = convertToNodeSpace( touchLocation );
    
		log("HelloWorld::onTouchEnded, pos: %f,%f -> %f,%f", touchLocation.x, touchLocation.y, nodePosition.x, nodePosition.y);
		PhysicsManager::getInstance()->MouseUp(nodePosition);
//		MouseUp(b2Vec2(nodePosition.x/PTM_RATIO,nodePosition.y/PTM_RATIO));
	}

}

void HelloWorld::onTouchMoved(Touch* touch, Event* event)
{
	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::MOVE_TYPE)
	{
		if(PhysicsManager::getInstance()->isMovingBody())
		{
			auto touchLocation = touch->getLocation();    
			auto nodePosition = convertToNodeSpace( touchLocation );
    
			log("HelloWorld::onTouchMoved, pos: %f,%f -> %f,%f", touchLocation.x, touchLocation.y, nodePosition.x, nodePosition.y);
			PhysicsManager::getInstance()->MouseMove(nodePosition);
//			MouseMove(b2Vec2(nodePosition.x/PTM_RATIO,nodePosition.y/PTM_RATIO));
		}
		else
		{
			auto delta = touch->getDelta();
			auto curPos = this->getPosition();
			this->setPosition(curPos + delta);
		}

	}
	if(PhysicsManager::getInstance()->getTouchType() == PhysicsManager::ADD_CUSTOM_TYPE)
	{
//		auto pos = this->convertToNodeSpace(touch->getLocation());
		if(_movingMark)
		{
			auto curPos = _movingMark->getPosition();
			_movingMark->setPosition(curPos + touch->getDelta());
		}
	}

}


void HelloWorld::addUI()
{
	float buttonScale = 2.0f;
	int titleFontSize = 24;

	Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

	_label = LabelTTF::create("Hello World", "Arial", 24);
    _label->setString("Mode: move");

    Button* move_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
    move_button->setName("Move");
	move_button->setTitleText("Move");
	move_button->setScale9Enabled(true);
	move_button->addTouchEventListener(CC_CALLBACK_2(HelloWorld::menuSetMoveTypeCallback,this));
	move_button->setContentSize(move_button->getContentSize()*buttonScale);
	move_button->setTitleFontSize(titleFontSize);

    Button* add_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
    add_button->setName("Add");
	add_button->setTitleText("Add");
	add_button->setScale9Enabled(true);
	add_button->addTouchEventListener(CC_CALLBACK_2(HelloWorld::menuSetAddTypeCallback,this));
	add_button->setContentSize(move_button->getContentSize());
	add_button->setTitleFontSize(titleFontSize);

    Button* add_custom_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
    add_custom_button->setName("Add Custom");
	add_custom_button->setTitleText("Add Custom");
	add_custom_button->setScale9Enabled(true);
	add_custom_button->addTouchEventListener(CC_CALLBACK_2(HelloWorld::menuSetAddCustomTypeCallback,this));
	add_custom_button->setContentSize(move_button->getContentSize());
	add_custom_button->setTitleFontSize(titleFontSize);

    Button* delete_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
    delete_button->setName("Delete");
	delete_button->setTitleText("Delete");
	delete_button->setScale9Enabled(true);
	delete_button->addTouchEventListener(CC_CALLBACK_2(HelloWorld::menuDeleteCallback,this));
	delete_button->setContentSize(move_button->getContentSize());
	delete_button->setTitleFontSize(titleFontSize);

    Button* confirm_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
    confirm_button->setName("Confirm");
	confirm_button->setTitleText("Confirm");
	confirm_button->setScale9Enabled(true);
	confirm_button->addTouchEventListener(CC_CALLBACK_2(HelloWorld::menuConfirmCallback,this));
	confirm_button->setContentSize(move_button->getContentSize());
	confirm_button->setTitleFontSize(titleFontSize);
    //Layout* opt_item = Layout::create();
    //opt_item->setTouchEnabled(true);
    //opt_item->setContentSize(add_button->getContentSize());
    //add_button->setPosition(Vec2(opt_item->getContentSize().width / 2.0f,
    //                                    opt_item->getContentSize().height / 2.0f));
    //opt_item->addChild(add_button);
	_label->setPosition(Vec2(visibleSize.width - _label->getContentSize().width/2, 
								visibleSize.height - _label->getContentSize().height/2)); 
	move_button->setPosition(Vec2(visibleSize.width - move_button->getContentSize().width/2,
								visibleSize.height - move_button->getContentSize().height/2 - _label->getContentSize().height));
	add_button->setPosition(Vec2(visibleSize.width - add_button->getContentSize().width/2,
								visibleSize.height - add_button->getContentSize().height/2 - _label->getContentSize().height
								- move_button->getContentSize().height));
	add_custom_button->setPosition(Vec2(visibleSize.width - add_custom_button->getContentSize().width/2,
								visibleSize.height - add_custom_button->getContentSize().height/2 - _label->getContentSize().height
								- move_button->getContentSize().height - add_button->getContentSize().height));
	auto pos = add_custom_button->getPosition();
	delete_button->setPosition(pos + Vec2(-delete_button->getContentSize().width, 0));
	pos = delete_button->getPosition();
	confirm_button->setPosition(pos + Vec2(-confirm_button->getContentSize().width, 0));


	this->addChild(_label);
	this->addChild(move_button);
	this->addChild(add_button);
	this->addChild(add_custom_button);
	this->addChild(delete_button);
	this->addChild(confirm_button);




    // Create the list view ex
    ListView* listView = ListView::create();
//    // set list view ex direction
//    listView->setDirection(ui::ScrollView::Direction::VERTICAL);
//    listView->setBounceEnabled(true);
//    listView->setBackGroundImage("cocosui/green_edit.png");
//    listView->setBackGroundImageScale9Enabled(true);
//    listView->setContentSize(Size(130, 600));
//    listView->setPosition(Vec2(visibleSize.width,
//                               (visibleSize.height - listView->getContentSize().height) / 2.0f ));
//    listView->addEventListener((ui::ListView::ccListViewCallback)CC_CALLBACK_2(HelloWorld::selectedItemEvent, this));
////    listView->addEventListener((ui::ListView::ccScrollViewCallback)CC_CALLBACK_2(UIListViewTest_Vertical::selectedItemEventScrollView,this));
//        
//    this->addChild(listView);

    // create model
    Button* default_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
    default_button->setName("Title Button");
	default_button->setTitleText("Test");
	default_button->setScale9Enabled(true);
        
    Layout* default_item = Layout::create();
    default_item->setTouchEnabled(true);
    default_item->setContentSize(default_button->getContentSize());
    default_button->setPosition(Vec2(default_item->getContentSize().width / 2.0f,
                                        default_item->getContentSize().height / 2.0f));
    default_item->addChild(default_button);
        
    //// set model
    //listView->setItemModel(default_item);
    //    
    //// add default item
    //ssize_t count = 14;
    //for (int i = 0; i < count / 4; ++i)
    //{
    //    listView->pushBackDefaultItem();
    //}
    //// insert default item
    //for (int i = 0; i < count / 4; ++i)
    //{
    //    listView->insertDefaultItem(0);
    //}
    //    Sprite* testSprite = Sprite::create("cocosui/backtotoppressed.png");
    //    testSprite->setPosition(Vec2(200,200));
    //    listView->addChild(testSprite);
        
    // add custom item
  //  for (int i = 0; i < count; ++i)
  //  {
  //      Button* custom_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
  //      custom_button->setName("Title Button");
		//custom_button->setTitleText(String::createWithFormat("Test%.2d", i + 3)->getCString());
  //      custom_button->setScale9Enabled(true);
  //      custom_button->setContentSize(default_button->getContentSize());
  //          
  //      Layout *custom_item = Layout::create();
  //      custom_item->setContentSize(custom_button->getContentSize());
  //      custom_button->setPosition(Vec2(custom_item->getContentSize().width / 2.0f, custom_item->getContentSize().height / 2.0f));
  //      custom_item->addChild(custom_button);
  //          
  //      listView->addChild(custom_item);
  //  }
    //// insert custom item
    //Vector<Widget*>& items = listView->getItems();
    //ssize_t items_count = items.size();
    //for (int i = 0; i < count / 4; ++i)
    //{
    //    Button* custom_button = Button::create("cocosui/button.png", "cocosui/buttonHighlighted.png");
    //    custom_button->setName("Title Button");
    //    custom_button->setScale9Enabled(true);
    //    custom_button->setContentSize(default_button->getContentSize());
    //        
    //    Layout *custom_item = Layout::create();
    //    custom_item->setContentSize(custom_button->getContentSize());
    //    custom_button->setPosition(Vec2(custom_item->getContentSize().width / 2.0f, custom_item->getContentSize().height / 2.0f));
    //    custom_item->addChild(custom_button);
    //    custom_item->setTag(1);
    //        
    //    listView->insertCustomItem(custom_item, items_count);
    //}

}

void HelloWorld::selectedItemEvent(Ref *pSender, ListView::EventType type)
{
    switch (type)
    {
        case cocos2d::ui::ListView::EventType::ON_SELECTED_ITEM_START:
        {
            ListView* listView = static_cast<ListView*>(pSender);
            CC_UNUSED_PARAM(listView);
            CCLOG("select child start index = %ld", listView->getCurSelectedIndex());
            break;
        }
        case cocos2d::ui::ListView::EventType::ON_SELECTED_ITEM_END:
        {
            ListView* listView = static_cast<ListView*>(pSender);
            CC_UNUSED_PARAM(listView);
            CCLOG("select child end index = %ld", listView->getCurSelectedIndex());
			PhysicsManager::getInstance()->setSideNum(listView->getCurSelectedIndex() + 3);
//			_sideNum = listView->getCurSelectedIndex() + 3;
            break;
        }
        default:
            break;
    }
}

void HelloWorld::onExit()
{
	Layer::onExit();
	PhysicsManager::purgeInstance();
}

void HelloWorld::addMark(const Vec2& pos)
{
	DrawNode* draw = DrawNode::create();
	draw->drawDot(Vec2(0, 0), markRadias, Color4F(0, 1, 0, 1));
	draw->drawDot(Vec2(0, 0), 5, Color4F(1, 1, 1, 1));
	draw->setPosition(pos);
	this->addChild(draw, 1);
	_marks.pushBack(draw);

}

DrawNode* HelloWorld::getMark(const Vec2& pos)
{
	ssize_t size = _marks.size();
	for(ssize_t i=0; i<size; i++)
	{
		auto mPos = _marks.at(i)->getPosition();
		if((mPos - pos).length()<= markRadias * 2) // make marks easy to catch.
			return _marks.at(i);
	}
	return nullptr;
}

void HelloWorld::addCustomPolygon()
{
	if(_marks.size()<3)
		return;
	std::vector<Vec2> points;
	ssize_t size = _marks.size();
	for(ssize_t i=0; i<size; i++)
	{
		points.push_back(_marks.at(i)->getPosition());
	}
	PhysicsManager::getInstance()->addCustomPolygon(points);
}