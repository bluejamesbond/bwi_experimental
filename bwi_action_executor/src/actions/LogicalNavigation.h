#ifndef bwi_actexec_LogicalNavigation_h__guard
#define bwi_actexec_LogicalNavigation_h__guard

#include "Action.h"

namespace bwi_actexec {

	
class LogicalNavigation : public bwi_actexec::Action {
public:
	
	LogicalNavigation(const std::string &name);
	
	void init(const std::vector<std::string>& params);
	
	int paramNumber() const {return 1;}
	
	std::string getName() const {return name;}
	
	void run();
	
	bool hasFinished() const {return done;}
	
	std::string toASP(unsigned int timeStep) const;
	
	Action *clone() const {return new LogicalNavigation(*this);}
	
private:
	std::string name;
	std::vector<std::string> parameters;
	bool done;

};	
}

#endif