#ifndef __MenuLayer__
#define __MenuLayer__
#include "cocos2d.h"
USING_NS_CC;

class MenuLayer: public Layer
{
public:
	bool init();
	CREATE_FUNC(MenuLayer);
	void addUI();
};

#endif