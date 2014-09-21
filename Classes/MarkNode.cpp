#include "MarkNode.h"

bool MarkNode::init()
{
	if(!DrawNode::init())
		return false;
	_text = ui::Text::create("0","",20);
	auto height = _text->getContentSize().height/2;
	_text->setPosition(Vec2(0, height));
	_text->setColor(Color3B(150, 0, 0));
	this->addChild(_text);
	return true;
}


void MarkNode::setNum(int num)
{
	_text->setString(String::createWithFormat("%d",num)->getCString());
}