#ifndef __MarkNode__
#define __MarkNode__
#include "cocos2d.h"
#include "ui/CocosGUI.h"
USING_NS_CC;

class MarkNode : public DrawNode
{
public:
	bool init();
	CREATE_FUNC(MarkNode);
	void setNum(int num);
private:
	ui::Text* _text;
};
#endif