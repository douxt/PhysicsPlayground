#include "PopMenu.h"



bool PopMenu::init()
{
	_listView = ui::ListView::create();
	_listView->setDirection(ui::ScrollView::Direction::VERTICAL);
	_listView->addEventListener((ui::ListView::ccListViewCallback)CC_CALLBACK_2(PopMenu::selectedItemEvent, this));
	this->addChild(_listView);
	_buttonScale = 1.5f;
	_titleFontSize = 18;
	_pos = Vec2(0, 0);
	_popTime = 0.3f;
	_isEntered = false;
	_isEntering = false;
	_margin = 0;
	this->setVisible(false);
	return true;
}

void PopMenu::selectedItemEvent(Ref *pSender, ListView::EventType type)
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
			ssize_t index = listView->getCurSelectedIndex();
            CCLOG("select child end index = %ld", index);
			auto item = listView->getItem(index);
			auto name = item->getChildren().at(0)->getName();
			if(_callbacks[name])
				_callbacks[name]();
			
//			PhysicsManager::getInstance()->setSideNum(listView->getCurSelectedIndex() + 3);
//			_sideNum = listView->getCurSelectedIndex() + 3;
            break;
        }
        default:
            break;
    }
}

void PopMenu::addButton(const std::string& name, std::function<void()> callback)
{
	_callbacks[name] = callback;
    Button* custom_button = Button::create("cocosui/backtotoppressed.png", "cocosui/backtotopnormal.png");
    custom_button->setName(name);
	custom_button->setTitleText(name);
    custom_button->setScale9Enabled(true);
    custom_button->setContentSize(custom_button->getContentSize() * _buttonScale);
	custom_button->setTitleFontSize(_titleFontSize);
	//auto func = [&](Ref* pSender, Widget::TouchEventType type){
	//	if(callback)
	//		callback();
	//};
	//custom_button->addTouchEventListener(func);
            
    Layout *custom_item = Layout::create();
    custom_item->setContentSize(custom_button->getContentSize());
    custom_button->setPosition(Vec2(custom_item->getContentSize().width / 2.0f, custom_item->getContentSize().height / 2.0f));
    custom_item->addChild(custom_button);
            
    _listView->addChild(custom_item);
	_listView->setContentSize(Size(custom_button->getContentSize().width, _listView->getChildrenCount()*custom_button->getContentSize().height));
}

void PopMenu::popEnter()
{
	if(this->getIsEntered() || this->getIsEntering())
	{
		return;
	}
	auto height = _listView->getContentSize().height;
	auto move = MoveBy::create(_popTime, Vec2(0, -height - _margin));
	auto fuc =[&](){
		this->_isEntering = false;
		this->_isEntered = true;
		auto zOrder = this->getLocalZOrder();
		this->setLocalZOrder(zOrder - 10);
	}; 
	auto callback = CallFunc::create(fuc);
	auto seq = Sequence::create(move, callback, nullptr);
	_listView->runAction(seq);
	this->setVisible(true);
	this->_isEntering=true;
	auto zOrder = this->getLocalZOrder();
	this->setLocalZOrder(zOrder + 10);
}

void PopMenu::popExit()
{
	if(_isEntered)
	{
		auto height = _listView->getContentSize().height;
		auto move = MoveBy::create(_popTime, Vec2(0, height + _margin));
		auto fuc =[&](){
			this->_isEntering = false;
			this->_isEntered = false;
			this->setVisible(false);
		}; 
		auto callback = CallFunc::create(fuc);
		auto seq = Sequence::create(move, callback, nullptr);
		_listView->runAction(seq);

		this->_isEntering=true;
	}
}

void PopMenu::popToggle()
{
	popEnter();
	popExit();
}

void PopMenu::setCallback(const std::string& name, std::function<void()> callback)
{
	_callbacks[name] = callback;
}

const Size& PopMenu::getListViewContentSize() const
{
	return _listView->getContentSize();
}