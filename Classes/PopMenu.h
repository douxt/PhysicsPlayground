#ifndef __PopMenu__
#define __PopMenu__
#include "cocos2d.h"
#include "ui/CocosGUI.h"

USING_NS_CC;
using namespace ui;

class PopMenu : public Node
{
public:
	bool init();
	CREATE_FUNC(PopMenu);
	void selectedItemEvent(Ref *pSender, ListView::EventType type);
	void addButton(const std::string& name, std::function<void()> callback = nullptr); 
	CC_SYNTHESIZE(Vec2, _pos, Pos);
	CC_SYNTHESIZE(float, _popTime, PopTime);
	CC_SYNTHESIZE_READONLY(bool, _isEntering, IsEntering);
	CC_SYNTHESIZE_READONLY(bool, _isEntered, IsEntered);
	CC_SYNTHESIZE(int, _margin, Margin);
	void popExit();
	void popEnter();
	void popToggle();
	void setCallback(const std::string& name, std::function<void()> callback);
	const Size& getListViewContentSize() const;
	void reName(const std::string& oldName, const std::string& newName);
private:
	ListView* _listView;
	Layout* _layout;
	std::unordered_map<std::string, std::function<void()>> _callbacks;
	float _buttonScale;
	int _titleFontSize;
	
//	Vec2 _pos;
};

#endif