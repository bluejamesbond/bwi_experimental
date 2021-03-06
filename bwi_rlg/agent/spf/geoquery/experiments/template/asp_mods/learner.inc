## Filter to skip long sentences during learning
type=filter.sentence.length id=learningFilter length=%{trainingMaxSentenceLength}
## Learners
type=learner.validation.perceptron id=perceptron data=train genlexbeam=%{lexbeam} iter=%{gIter} validator=validator hard=true filter=learningFilter genlex=genlex conflateParses=true errorDriven=true
type=learner.validation.stocgrad id=stocgrad data=train genlexbeam=%{lexbeam} iter=%{gIter} validator=validator filter=learningFilter genlex=genlex conflateParses=false errorDriven=true 
