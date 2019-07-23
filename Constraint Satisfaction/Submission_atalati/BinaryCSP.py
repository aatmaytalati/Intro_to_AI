from collections import deque


#By Aatmay S. Talati
#Project 2 - CS 3600
#Due Date : JUNE 18, 2017, 11:59 PM
#Test Case score: 18/18 + 0/2 (Extra Credit)


"""
    Base class for unary constraints
    Implement isSatisfied in subclass to use
"""
class UnaryConstraint:
    def __init__(self, var):
        self.var = var

    def isSatisfied(self, value):
        util.raiseNotDefined()

    def affects(self, var):
        return var == self.var


""" 
    Implementation of UnaryConstraint
    Satisfied if value does not match passed in paramater
"""
class BadValueConstraint(UnaryConstraint):
    def __init__(self, var, badValue):
        self.var = var
        self.badValue = badValue

    def isSatisfied(self, value):
        return not value == self.badValue

    def __repr__(self):
        return 'BadValueConstraint (%s) {badValue: %s}' % (str(self.var), str(self.badValue))


""" 
    Implementation of UnaryConstraint
    Satisfied if value matches passed in paramater
"""
class GoodValueConstraint(UnaryConstraint):
    def __init__(self, var, goodValue):
        self.var = var
        self.goodValue = goodValue

    def isSatisfied(self, value):
        return value == self.goodValue

    def __repr__(self):
        return 'GoodValueConstraint (%s) {goodValue: %s}' % (str(self.var), str(self.goodValue))


"""
    Base class for binary constraints
    Implement isSatisfied in subclass to use
"""
class BinaryConstraint:
    def __init__(self, var1, var2):
        self.var1 = var1
        self.var2 = var2

    def isSatisfied(self, value1, value2):
        util.raiseNotDefined()

    def affects(self, var):
        return var == self.var1 or var == self.var2

    def otherVariable(self, var):
        if var == self.var1:
            return self.var2
        return self.var1


"""
    Implementation of BinaryConstraint
    Satisfied if both values assigned are different
"""
class NotEqualConstraint(BinaryConstraint):
    def isSatisfied(self, value1, value2):
        if value1 == value2:
            return False
        return True

    def __repr__(self):
        return 'BadValueConstraint (%s, %s)' % (str(self.var1), str(self.var2))


class ConstraintSatisfactionProblem:
    """
    Structure of a constraint satisfaction problem.
    Variables and domains should be lists of equal length that have the same order.
    varDomains is a dictionary mapping variables to possible domains.

    Args:
        variables (list<string>): a list of variable names
        domains (list<set<value>>): a list of sets of domains for each variable
        binaryConstraints (list<BinaryConstraint>): a list of binary constraints to satisfy
        unaryConstraints (list<BinaryConstraint>): a list of unary constraints to satisfy
    """
    def __init__(self, variables, domains, binaryConstraints = [], unaryConstraints = []):
        self.varDomains = {}
        for i in xrange(len(variables)):
            self.varDomains[variables[i]] = domains[i]
        self.binaryConstraints = binaryConstraints
        self.unaryConstraints = unaryConstraints

    def __repr__(self):
        return '---Variable Domains\n%s---Binary Constraints\n%s---Unary Constraints\n%s' % ( \
            ''.join([str(e) + ':' + str(self.varDomains[e]) + '\n' for e in self.varDomains]), \
            ''.join([str(e) + '\n' for e in self.binaryConstraints]), \
            ''.join([str(e) + '\n' for e in self.binaryConstraints]))


class Assignment:
    """
    Representation of a partial assignment.
    Has the same varDomains dictionary stucture as ConstraintSatisfactionProblem.
    Keeps a second dictionary from variables to assigned values, with None being no assignment.

    Args:
        csp (ConstraintSatisfactionProblem): the problem definition for this assignment
    """
    def __init__(self, csp):
        self.varDomains = {}
        for var in csp.varDomains:
            self.varDomains[var] = set(csp.varDomains[var])
        self.assignedValues = { var: None for var in self.varDomains }

    """
    Determines whether this variable has been assigned.

    Args:
        var (string): the variable to be checked if assigned
    Returns:
        boolean
        True if var is assigned, False otherwise
    """
    def isAssigned(self, var):
        return self.assignedValues[var] != None

    """
    Determines whether this problem has all variables assigned.

    Returns:
        boolean
        True if assignment is complete, False otherwise
    """
    def isComplete(self):
        for var in self.assignedValues:
            if not self.isAssigned(var):
                return False
        return True

    """
    Gets the solution in the form of a dictionary.

    Returns:
        dictionary<string, value>
        A map from variables to their assigned values. None if not complete.
    """
    def extractSolution(self):
        if not self.isComplete():
            return None
        return self.assignedValues

    def __repr__(self):
        return '---Variable Domains\n%s---Assigned Values\n%s' % ( \
            ''.join([str(e) + ':' + str(self.varDomains[e]) + '\n' for e in self.varDomains]), \
            ''.join([str(e) + ':' + str(self.assignedValues[e]) + '\n' for e in self.assignedValues]))



####################################################################################################


"""
    Checks if a value assigned to a variable is consistent with all binary constraints in a problem.
    Do not assign value to var. Only check if this value would be consistent or not.
    If the other variable for a constraint is not assigned, then the new value is consistent with the constraint.

    Args:
        assignment (Assignment): the partial assignment
        csp (ConstraintSatisfactionProblem): the problem definition
        var (string): the variable that would be assigned
        value (value): the value that would be assigned to the variable
    Returns:
        boolean
        True if the value would be consistent with all currently assigned values, False otherwise
"""
def consistent(assignment, csp, var, value):
    """Question 1"""
    """YOUR CODE HERE"""
    
    #binary Constraints
    TempB = 0
    c = csp
    assi = assignment
    d = assi.assignedValues
    e = assi.isAssigned
   
    
    for TempB in c.binaryConstraints:
        if not TempB.isSatisfied(d[TempB.otherVariable(var)], value):
            if e(TempB.otherVariable(var)) and TempB.affects(var):
                    return not True 
    
    #Unary Constraint
    TempU = 0
    c = csp
    assi = assignment

    for TempU in c.unaryConstraints:
        if TempU.isSatisfied(value) != True and TempU.var == var:
                return not True
    return not False


"""
    Recursive backtracking algorithm.
    A new assignment should not be created. The assignment passed in should have its domains updated with inferences.
    In the case that a recursive call returns failure or a variable assignment is incorrect, the inferences made along
    the way should be reversed. See maintainArcConsistency and forwardChecking for the format of inferences.

    Examples of the functions to be passed in:
    orderValuesMethod: orderValues, leastConstrainingValuesHeuristic
    selectVariableMethod: chooseFirstVariable, minimumRemainingValuesHeuristic
    inferenceMethod: noInferences, maintainArcConsistency, forwardChecking

    Args:
        assignment (Assignment): a partial assignment to expand upon
        csp (ConstraintSatisfactionProblem): the problem definition
        orderValuesMethod (function<assignment, csp, variable> returns list<value>): a function to decide the next value to try
        selectVariableMethod (function<assignment, csp> returns variable): a function to decide which variable to assign next
    Returns:
        Assignment
        A completed and consistent assignment. None if no solution exists.
"""
def recursiveBacktracking(assignment, csp, orderValuesMethod, selectVariableMethod):
    """Question 1"""
    """YOUR CODE HERE"""
    c = csp
    r = recursiveBacktracking
    CTempVar = selectVariableMethod(assignment, c)
    assi = assignment
      
    if assi.isComplete(): 
        return assi

    else:  
        a = orderValuesMethod   
        b = selectVariableMethod
        c = csp
        assi = assignment
        CTempVar = b(assi, c)
       
        for valz in a(assi, c, CTempVar):
            if consistent(assi, c, CTempVar, valz):
                assi.assignedValues[CTempVar] = valz
                    
                if r(assi, c, a, b) == None:
                    assi.assignedValues[CTempVar] = None
              
                elif r(assi, c, a, b) != None:
                    if assi.assignedValues[CTempVar] != None:
                        return assi
            
        return None

  

"""
    Uses unary constraints to eleminate values from an assignment.

    Args:
        assignment (Assignment): a partial assignment to expand upon
        csp (ConstraintSatisfactionProblem): the problem definition
    Returns:
        Assignment
        An assignment with domains restricted by unary constraints. None if no solution exists.
"""
def eliminateUnaryConstraints(assignment, csp):
    domains = assignment.varDomains
    for var in domains:
        for constraint in (c for c in csp.unaryConstraints if c.affects(var)):
            for value in (v for v in list(domains[var]) if not constraint.isSatisfied(v)):
                domains[var].remove(value)
                if len(domains[var]) == 0:
                    # Failure due to invalid assignment
                    return None
    return assignment


"""
    Trivial method for choosing the next variable to assign.
    Uses no heuristics.
"""

def chooseFirstVariable(assignment, csp):
    for var in csp.varDomains:
        if not assignment.isAssigned(var):
            return var


"""
    Selects the next variable to try to give a value to in an assignment.
    Uses minimum remaining values heuristic to pick a variable. Use degree heuristic for breaking ties.

    Args:
        assignment (Assignment): the partial assignment to expand
        csp (ConstraintSatisfactionProblem): the problem description
    Returns:
        the next variable to assign
""" 
def minimumRemainingValuesHeuristic(assignment, csp):

    nextVar = None
    domains = assignment.varDomains
    Q2Deg = 0
    """Question 2"""
    """YOUR CODE HERE"""
    mrv = float('inf')
    for var in domains:
        Q2CurDeg = 0
        if not assignment.isAssigned(var):
          
            for bc in csp.binaryConstraints:
                if bc.affects(var):
                    if not assignment.isAssigned(bc.otherVariable(var)):
                        Q2CurDeg += 1
            MRVQ2 = len(domains[var])
      
            if MRVQ2 < mrv:
                mrv = MRVQ2
                nextVar = var
                Q2Deg = Q2CurDeg
            
            if MRVQ2 == mrv and Q2CurDeg > Q2Deg:
                mrv = MRVQ2
                nextVar = var
                Q2Deg = Q2CurDeg
    return nextVar


"""
    Trivial method for ordering values to assign.
    Uses no heuristics.
"""
def orderValues(assignment, csp, var):
    return list(assignment.varDomains[var])


"""
    Creates an ordered list of the remaining values left for a given variable.
    Values should be attempted in the order returned.
    The least constraining value should be at the front of the list.

    Args:
        assignment (Assignment): the partial assignment to expand
        csp (ConstraintSatisfactionProblem): the problem description
        var (string): the variable to be assigned the values
    Returns:
        list<values>
        a list of the possible values ordered by the least constraining value heuristic
"""
def leastConstrainingValuesHeuristic(assignment, csp, var):
	#values = list(assignment.varDomains[var])
	"""Hint: Creating a helper function to count the number of constrained values might be useful"""
	"""Question 3"""
	"""YOUR CODE HERE"""
	MyList = []
	for value in assignment.varDomains[var]:
                affected = 0
                for binConstraint in (tempVar for tempVar in csp.binaryConstraints if tempVar.affects(var)):
                        otherVar = binConstraint.otherVariable(var)

                        for tempVarP in assignment.varDomains[otherVar]:
                            if not binConstraint.isSatisfied(value, tempVarP):
                                for possibleValue in tempVarP:
                                    affected += 1
		MyList = MyList + [(affected, value)]
	MyList.sort()
	return [GetSecondElementofMyList[1] for GetSecondElementofMyList in MyList]


"""
    Trivial method for making no inferences.
"""
def noInferences(assignment, csp, var, value):
    return set([])


"""
    Implements the forward checking algorithm.
    Each inference should take the form of (variable, value) where the value is being removed from the
    domain of variable. This format is important so that the inferences can be reversed if they
    result in a conflicting partial assignment. If the algorithm reveals an inconsistency, any
    inferences made should be reversed before ending the fuction.

    Args:
        assignment (Assignment): the partial assignment to expand
        csp (ConstraintSatisfactionProblem): the problem description
        var (string): the variable that has just been assigned a value
        value (string): the value that has just been assigned
    Returns:
        set<tuple<variable, value>>
        the inferences made in this call or None if inconsistent assignment
"""

def forwardChecking(assignment, csp, var, value):
    inferences = set([])
    infL = []
    a = infL.append
    d = assignment.varDomains
    incF = 0
    
    
    """Question 4"""
    """YOUR CODE HERE"""
    for tmpfc in csp.binaryConstraints:
        TempFcV = tmpfc.otherVariable(var)
        if tmpfc.affects(var) and value in d[TempFcV]:
            if not tmpfc.isSatisfied(assignment.assignedValues[var], value):
                d[TempFcV].remove(value)
                a((TempFcV, value))
                if not len(d[TempFcV]):
                    incF = True
                    break
    if incF:
        for tuple_var, tuple_val in infL:
            d[tuple_var].update(tuple_val)
        return None
    inferences = set(infL)
    return inferences

"""
    Recursive backtracking algorithm.
    A new assignment should not be created. The assignment passed in should have its domains updated with inferences.

    In the case that a recursive call returns failure or a variable assignment is incorrect, the inferences made along
    the way should be reversed. See maintainArcConsistency and forwardChecking for the format of inferences.


    Examples of the functions to be passed in:
    orderValuesMethod: orderValues, leastConstrainingValuesHeuristic
    selectVariableMethod: chooseFirstVariable, minimumRemainingValuesHeuristic
    inferenceMethod: noInferences, maintainArcConsistency, forwardChecking


    Args:
        assignment (Assignment): a partial assignment to expand upon
        csp (ConstraintSatisfactionProblem): the problem definition
        orderValuesMethod (function<assignment, csp, variable> returns list<value>): a function to decide the next value to try
        selectVariableMethod (function<assignment, csp> returns variable): a function to decide which variable to assign next
        inferenceMethod (function<assignment, csp, variable, value> returns set<variable, value>): a function to specify what type of inferences to use
                Can be forwardChecking or maintainArcConsistency
    Returns:
        Assignment

        A completed and consistent assignment. None if no solution exists.
"""
def recursiveBacktrackingWithInferences(assignment, csp, orderValuesMethod, selectVariableMethod, inferenceMethod):
    
    """Question 4"""
    """YOUR CODE HERE"""
    svarM = selectVariableMethod
    orvM = orderValuesMethod
    assi = assignment
    a = assi.assignedValues
    b = assi.isComplete
    
    if b(): #checking if its complete?
        return assi #if yes then return assignemnt 
    else: #else continue
        VarC = svarM(assi, csp)
        for valz in orvM(assi, csp, VarC):
            if consistent(assi, csp, VarC, valz):
                a[VarC] = valz
                inferencesSet = inferenceMethod(assi, csp, VarC, valz)
                if inferencesSet != None: 
                    inferencesList = list(inferencesSet)
                    a[VarC] = valz
                
                    if recursiveBacktrackingWithInferences(assi, csp, orvM, selectVariableMethod, inferenceMethod) != None:
                        return assi
                    else:
                        for var, val in inferencesList:
                            assi.varDomains[var].update(val)
                        a[VarC] = None
                else:
                    continue
    return None



"""
    Helper funciton to maintainArcConsistency and AC3.
    Remove values from var2 domain if constraint cannot be satisfied.
    Each inference should take the form of (variable, value) where the value is being removed from the
    domain of variable. This format is important so that the inferences can be reversed if they
    result in a conflicting partial assignment. If the algorithm reveals an inconsistency, any
    inferences made should be reversed before ending the fuction.

    Args:
        assignment (Assignment): the partial assignment to expand
        csp (ConstraintSatisfactionProblem): the problem description
        var1 (string): the variable with consistent values
        var2 (string): the variable that should have inconsistent values removed
        constraint (BinaryConstraint): the constraint connecting var1 and var2
    Returns:
        set<tuple<variable, value>>
        the inferences made in this call or None if inconsistent assignment
"""
def revise(assignment, csp, var1, var2, constraint):

    inferences = set([])
    
    inferencesList = []
    inconsistencyFlag = 0
    currRemove = 0 
    removeList = []
    """Question 5"""
    """YOUR CODE HERE"""
    
    for x in assignment.varDomains[var2]:
        currRemove = 1
        
        for y in assignment.varDomains[var1]:
            if constraint.isSatisfied(x, y):
                currRemove = 0
                break
        if currRemove:
           removeList.append(x)
         
    for x in removeList:
        assignment.varDomains[var2].remove(x)
        inferencesList.append((var2, x))
        if not len(assignment.varDomains[var2]):
            inconsistencyFlag = 1
            break
    
    if inconsistencyFlag:
        for var, val in inferencesList:
            assignment.varDomains[var].update(val)
        return None
    else:
        inferences = set(inferencesList)
    return inferences


"""
    Implements the maintaining arc consistency algorithm.
    Inferences take the form of (variable, value) where the value is being removed from the
    domain of variable. This format is important so that the inferences can be reversed if they
    result in a conflicting partial assignment. If the algorithm reveals an inconsistency, and
    inferences made should be reversed before ending the fuction.

    Args:
        assignment (Assignment): the partial assignment to expand
        csp (ConstraintSatisfactionProblem): the problem description
        var (string): the variable that has just been assigned a value
        value (string): the value that has just been assigned
    Returns:
        set<<variable, value>>
        the inferences made in this call or None if inconsistent assignment
"""
from itertools import chain
def maintainArcConsistency(assignment, csp, var, value):
    #print 'called MAC'
    inferences = set([])
    domains = assignment.varDomains
    """Hint: implement revise first and use it as a helper function"""
    """Question 5""" 
    """YOUR CODE HERE"""
      
    arcQueue = populate(deque(), csp, var)
    while len(arcQueue):
        head, tail = arcQueue.popleft()
        currBc = None
        for bc in csp.binaryConstraints:
            if bc.affects(head) and bc.affects(tail):
                currBc = bc

        inferencesSet = revise(assignment, csp, head, tail, currBc)
        if inferencesSet == None:
            for reverseVar, reverseVal in list(inferences):
                domains[reverseVar].update(reverseVal)
            return None
        if len(inferencesSet) != 0:
            for bc in csp.binaryConstraints:
                if bc.affects(tail):
                    arcQueue.append((tail, bc.otherVariable(tail)))
        inferences = set(chain(inferences, inferencesSet))
    return inferences

def populate(queue, csp, var):
    for bc in csp.binaryConstraints:
        if bc.affects(var):
            queue.append((var, bc.otherVariable(var)))
    return queue
"""
	AC3 algorithm for constraint propogation. Used as a preprocessing step to reduce the problem
	before running recursive backtracking.

	Args:
		assignment (Assignment): the partial assignment to expand
		csp (ConstraintSatisfactionProblem): the problem description
	Returns:
		Assignment
		the updated assignment after inferences are made or None if an inconsistent assignment
"""
def AC3(assignment, csp):
        inferences = set([])
	"""Hint: implement revise first and use it as a helper function"""
	"""Question 6"""
	"""YOUR CODE HERE"""
   # a = assignment.varDomains
        templist = []

        for tempX in csp.binaryConstraints:
            templist.append((tempX.var1, tempX.var2, tempX))
            templist.append((tempX.var2, tempX.var1, tempX)) 

        for var1, var2, var3 in templist:
            infer = revise(assignment, csp, var1, var2, var3)
        
            if infer == None:
                return None
            else:
                if len(infer) != 0:
                    for tempZ in csp.binaryConstraints:
                        if tempZ.affects(var2):
                            templist.append((var2, tempZ.otherVariable(var2), tempZ))
	return assignment


"""
	Solves a binary constraint satisfaction problem.

	Args:
		csp (ConstraintSatisfactionProblem): a CSP to be solved
		orderValuesMethod (function): a function to decide the next value to try
		selectVariableMethod (function): a function to decide which variable to assign next
		inferenceMethod (function): a function to specify what type of inferences to use
		useAC3 (boolean): specifies whether to use the AC3 preprocessing step or not
	Returns:
		dictionary<string, value>
		A map from variables to their assigned values. None if no solution exists.
"""
def solve(csp, orderValuesMethod=leastConstrainingValuesHeuristic, selectVariableMethod=minimumRemainingValuesHeuristic, inferenceMethod=None, useAC3=True):
	assignment = Assignment(csp)

	assignment = eliminateUnaryConstraints(assignment, csp)
	if assignment == None:
		return assignment

	if useAC3:
		assignment = AC3(assignment, csp)
		if assignment == None:
			return assignment
	if inferenceMethod is None or inferenceMethod==noInferences:
		assignment = recursiveBacktracking(assignment, csp, orderValuesMethod, selectVariableMethod)
	else:
		assignment = recursiveBacktrackingWithInferences(assignment, csp, orderValuesMethod, selectVariableMethod, inferenceMethod)
	if assignment == None:
		return assignment

	return assignment.extractSolution()
