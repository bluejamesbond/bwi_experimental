
#ifndef actasp_Clingo_h__guard
#define actasp_Clingo_h__guard

#include <actasp/AspKR.h>
#include <actasp/MultiPolicy.h>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <set>

namespace actasp {

class Action;

class Clingo : public AspKR {
public:

	Clingo(unsigned int max_n,
	       const std::string& queryDir,
	       const std::string& domainDir,
	       const ActionSet& actions,
         unsigned int max_time = 0
        ) throw();

	AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
	
	bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
	
	bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();
  
  void reset() throw();

	AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw ();
	
	std::vector< AnswerSet> computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw ();
	
	MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
	
	void setMaxTimeStep(unsigned int max_n) throw() {
		this->max_n = max_n;
  }

private:

	unsigned int max_n;
  unsigned int max_time;
	std::string queryDir;
	std::string domainDir;
  ActionSet allActions;
  std::string actionFilter;

	std::string generatePlanQuery(	const std::vector<actasp::AspRule>& goalRules, 
									unsigned int timeStep, 
									bool filterActions) const throw();

	std::vector<actasp::AnswerSet> krQuery(	const std::string& query, 
											unsigned int timeStep,
											const std::string& fileName, 
											unsigned int answerSetsNumber) const throw();
                      

};

}
#endif
