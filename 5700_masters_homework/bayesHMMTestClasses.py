# bayesHMMTestClasses.py
# ------------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


import testClasses
import bayesNet
import random
import layout
import hunters
from copy import deepcopy
from tempfile import mkstemp
import time
from shutil import move
from os import remove, close
import util
from util import manhattanDistance
import busters
from game import Agent
from game import Actions
from game import Directions
import re

class GraphEqualityTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(GraphEqualityTest, self).__init__(question, testDict)
        layoutText = testDict['layout']
        self.layoutName = testDict['layoutName']

        lay = layout.Layout([row.strip() for row in layoutText.split('\n')])
        self.startState = hunters.GameState()
        self.startState.initialize(lay, 0)

    def getEmptyStudentBayesNet(self, moduleDict):
        inferenceModule = moduleDict['inference']
        studentComputation = inferenceModule.constructBayesNet
        net = studentComputation(self.startState)
        return net

    def execute(self, grades, moduleDict, solutionDict):
        # load student code and staff code solutions
        studentNet = self.getEmptyStudentBayesNet(moduleDict)
        goldNet = bayesNet.constructEmptyBayesNetFromString(solutionDict['solutionString'])
        correct = studentNet.sameGraph(goldNet)
        sameValues = studentNet.sameValuesDict(goldNet)
        if correct and sameValues:
            return self.testPass(grades)
        self.addMessage('Bayes net graphs are not equal.')
        missingVars = goldNet.variablesSet() - studentNet.variablesSet()
        extraVars = studentNet.variablesSet() - goldNet.variablesSet()
        if missingVars:
            self.addMessage('Student solution is missing variables: ' + str(missingVars) + '\n')
        if extraVars:
            self.addMessage('Student solution has extra variables: ' + str(extraVars) + '\n')
        studentEdges = set([str(fromVar) + " -> " + str(toVar) for toVar in studentNet.variablesSet() for fromVar in studentNet.inEdges()[toVar]])
        goldEdges = set([str(fromVar) + " -> " + str(toVar) for toVar in goldNet.variablesSet() for fromVar in goldNet.inEdges()[toVar]])
        missingEdges = goldEdges - studentEdges
        extraEdges = studentEdges - goldEdges
        if missingEdges:
            self.addMessage('Student solution is missing edges:')
            for edge in sorted(missingEdges):
                self.addMessage('    ' + str(edge))
            self.addMessage('\n')
        if extraEdges:
            self.addMessage('Student solution has extra edges:')
            for edge in sorted(extraEdges):
                self.addMessage('    ' + str(edge))
            self.addMessage('\n')
        if not sameValues:
            self.addMessage('Student solution has incorrect values dictionary.')
            studentDict = studentNet.variableDomainsDict()
            goldDict = goldNet.variableDomainsDict()
            missingDictVars = set(goldDict) - set(studentDict)
            extraDictVars = set(studentDict) - set(goldDict)
            if missingDictVars:
                self.addMessage('Student dictionary is missing variables: ' + str(missingDictVars))
            if extraDictVars:
                self.addMessage('Student dictionary has extra variables: ' + str(extraDictVars))
            for variable, assignments in goldDict.items():
                if variable not in studentDict:
                    continue
                studentAssignments = studentDict[variable]
                missing = set(assignments) - set(studentAssignments)
                extra = set(studentAssignments) - set(assignments)
                if missing:
                    self.addMessage('Student dictionary for ' + variable + ' is missing assignments: ' + str(missing))
                if extra:
                    self.addMessage('Student dictionary for ' + variable + ' has extra assignments: ' + str(extra))
        return self.testFail(grades)

        
    def writeSolution(self, moduleDict, filePath):
        inferenceModule = moduleDict['inference']
        with open(filePath, 'w') as handle:
            handle.write('# This is the solution file for %s.\n\nsolutionString: """\n' % self.path)
            net = inferenceModule.constructBayesNet(self.startState)
            handle.write(str(net))
            handle.write('\n"""\n')
        return True

    def createPublicVersion(self):
        pass

class BayesNetEqualityTest(GraphEqualityTest):

    def execute(self, grades, moduleDict, solutionDict):
        # load student code and staff code solutions
        studentNet = self.getEmptyStudentBayesNet(moduleDict)
        goldNet = parseSolutionBayesNet(solutionDict)
        if not studentNet.sameGraph(goldNet):
            self.addMessage('Bayes net graphs are not equivalent. Please check that your Q1 implementation is correct.')
            return self.testFail(grades)
        moduleDict['bayesAgents'].fillCPTs(studentNet, self.startState)
        for variable in goldNet.variablesSet():
            try: 
                studentFactor = studentNet.getCPT(variable)
            except KeyError:
                self.addMessage('Student Bayes net missing CPT for variable ' + str(variable))
                return self.testFail(grades)
            goldFactor = goldNet.getCPT(variable)
            if not studentFactor == goldFactor:
                self.addMessage('First factor in which student answer differs from solution: P({} | {})'.format(studentFactor.unconditionedVariables(), studentFactor.conditionedVariables()))
                self.addMessage('Student Factor:\n' + str(studentFactor))
                self.addMessage('Correct Factor:\n' + str(goldFactor))
                return self.testFail(grades)
        return self.testPass(grades)

    def writeSolution(self, moduleDict, filePath):
        bayesAgentsModule = moduleDict['bayesAgents']
        with open(filePath, 'w') as handle:
            handle.write('# This is the solution file for %s.\n\n' % self.path)
            net, _ = bayesAgentsModule.constructBayesNet(self.startState)
            bayesAgentsModule.fillCPTs(net, self.startState)
            handle.write(net.easierToParseString(printVariableDomainsDict=True))
        return True

class FactorEqualityTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(FactorEqualityTest, self).__init__(question, testDict)
        self.seed = self.testDict['seed']
        random.seed(self.seed)
        self.alg = self.testDict['alg']
        self.max_points = int(self.testDict['max_points'])
        self.testPath = testDict['path']
        self.constructRandomly = testDict['constructRandomly']

    def execute(self, grades, moduleDict, solutionDict):
        # load student code and staff code solutions
        studentFactor = self.solveProblem(moduleDict)
        goldenFactor = parseFactorFromFileDict(solutionDict)

        # compare computed factor to stored factor
        self.addMessage('Executed FactorEqualityTest')
        if studentFactor == goldenFactor:
            # extra condition for test passing for this test type:
            if self.alg == 'inferenceByVariableElimination':
                goldenCallTrackingList = eval(solutionDict['callTrackingList'])
                if self.callTrackingList != goldenCallTrackingList:
                    self.addMessage('Order of joining by variables and elimination by variables is incorrect for variable elimination')
                    self.addMessage('Student performed the following operations in order: ' + str(self.callTrackingList) + '\n')
                    self.addMessage('Correct order of operations: ' + str(goldenCallTrackingList) + '\n')
                    return self.testFail(grades)

            return self.testPass(grades)
        else:
            self.addMessage('Factors are not equal.\n')
            self.addMessage('Student generated factor:\n\n' + str(studentFactor) + '\n\n')
            self.addMessage('Correct factor:\n\n' + str(goldenFactor) + '\n')

            studentProbabilityTotal = sum([studentFactor.getProbability(assignmentDict) for assignmentDict in studentFactor.getAllPossibleAssignmentDicts()])
            correctProbabilityTotal = sum([goldenFactor.getProbability(assignmentDict) for assignmentDict in goldenFactor.getAllPossibleAssignmentDicts()])
            if abs(studentProbabilityTotal - correctProbabilityTotal) > 10e-12:
                self.addMessage('Sum of probability in student generated factor is not the same as in correct factor')
                self.addMessage('Student sum of probability: ' + str(studentProbabilityTotal))
                self.addMessage('Correct sum of probability: ' + str(correctProbabilityTotal))

            return self.testFail(grades)


    def writeSolution(self, moduleDict, filePath):

        if self.constructRandomly:
            if self.alg == 'joinFactors' or self.alg == 'eliminate' or \
                    self.alg == 'normalize':
                replaceTestFile(self.testPath, "Factors", self.factorsDict)
            elif self.alg == 'inferenceByVariableElimination' or \
                    self.alg == 'inferenceByLikelihoodWeightingSampling':
                replaceTestFile(self.testPath, "BayesNet", self.problemBayesNet)

        factor = self.solveProblem(moduleDict)
        with open(filePath, 'w') as handle:
            handle.write('# This is the solution file for %s.\n' % self.path)
            printString = factor.easierToParseString()
            handle.write('%s\n' % (printString))

            if self.alg == 'inferenceByVariableElimination':
                handle.write('callTrackingList: "' + repr(self.callTrackingList) + '"\n')
        return True


class FactorInputFactorEqualityTest(FactorEqualityTest):
    def __init__(self, question, testDict):
        super(FactorInputFactorEqualityTest, self).__init__(question, testDict)
        self.factorArgs = self.testDict['factorArgs']
        eliminateToPerform = (self.alg == 'eliminate')
        evidenceAssignmentToPerform = (self.alg == 'normalize')

        parseDict =  parseFactorInputProblem(testDict, goingToEliminate=eliminateToPerform,
                                             goingToEvidenceAssign=evidenceAssignmentToPerform)
        self.variableDomainsDict = parseDict['variableDomainsDict']
        self.factorsDict = parseDict['factorsDict']
        if eliminateToPerform:
            self.eliminateVariable = parseDict['eliminateVariable']
        if evidenceAssignmentToPerform:
            self.evidenceDict = parseDict['evidenceDict']
        self.max_points = int(self.testDict['max_points'])

    def solveProblem(self, moduleDict):
        factorOperationsModule =  moduleDict['factorOperations']
        studentComputation = getattr(factorOperationsModule, self.alg)
        if self.alg == 'joinFactors':
            solvedFactor = studentComputation(self.factorsDict.values())
        elif self.alg == 'eliminate':
            solvedFactor = studentComputation(list(self.factorsDict.values())[0],
                                              self.eliminateVariable)
        elif self.alg == 'normalize':
            newVariableDomainsDict = deepcopy(self.variableDomainsDict)
            for variable, value in self.evidenceDict.items():
                newVariableDomainsDict[variable] = [value]
            origFactor = list(self.factorsDict.values())[0]
            specializedFactor = origFactor.specializeVariableDomains(newVariableDomainsDict)
            solvedFactor = studentComputation(specializedFactor)
        
        return solvedFactor


class BayesNetInputFactorEqualityTest(FactorEqualityTest):

    def __init__(self, question, testDict):
        super(BayesNetInputFactorEqualityTest, self).__init__(question, testDict)

        parseDict = parseBayesNetProblem(testDict)

        self.queryVariables = parseDict['queryVariables']
        self.evidenceDict = parseDict['evidenceDict']

        if self.alg == 'inferenceByVariableElimination':
            self.callTrackingList = []
            self.variableEliminationOrder = parseDict['variableEliminationOrder']
        elif self.alg == 'inferenceByLikelihoodWeightingSampling':
            self.numSamples = parseDict['numSamples']

        self.problemBayesNet = parseDict['problemBayesNet']
        self.max_points = int(self.testDict['max_points'])

    def solveProblem(self, moduleDict):
        inferenceModule = moduleDict['inference']
        if self.alg == 'inferenceByVariableElimination':
            studentComputationWithCallTracking = getattr(inferenceModule, self.alg + 'WithCallTracking')
            studentComputation = studentComputationWithCallTracking(self.callTrackingList)
            solvedFactor = studentComputation(self.problemBayesNet, self.queryVariables, self.evidenceDict, self.variableEliminationOrder)
        elif self.alg == 'inferenceByLikelihoodWeightingSampling':
            randomSource = util.FixedRandom().random
            studentComputationRandomSource = getattr(inferenceModule, self.alg + 'RandomSource')
            studentComputation = studentComputationRandomSource(randomSource)
            #random.seed(self.seed) # reset seed so that if we had to compute the bayes net we still have the initial seed
            solvedFactor = studentComputation(self.problemBayesNet, self.queryVariables, self.evidenceDict, self.numSamples)
        
        return solvedFactor

class MostLikelyFoodHousePositionTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(MostLikelyFoodHousePositionTest, self).__init__(question, testDict)
        layoutText = testDict['layout']
        self.layoutName = testDict['layoutName']

        lay = layout.Layout([row.strip() for row in layoutText.split('\n')])
        self.startState = hunters.GameState()
        self.startState.initialize(lay, 0)

        self.evidence = eval(testDict['evidence'])
        self.eliminationOrder = eval(testDict['eliminationOrder'])

    def execute(self, grades, moduleDict, solutionDict):
        # load student code and staff code solutions
        bayesAgentsModule = moduleDict['bayesAgents']
        FOOD_HOUSE_VAR = bayesAgentsModule.FOOD_HOUSE_VAR
        studentBayesNet, _ = bayesAgentsModule.constructBayesNet(self.startState)
        bayesAgentsModule.fillCPTs(studentBayesNet, self.startState)
        studentFunction = bayesAgentsModule.getMostLikelyFoodHousePosition
        studentPosition = studentFunction(self.evidence, studentBayesNet, self.eliminationOrder)[FOOD_HOUSE_VAR]
        goldPosition = solutionDict['answer']
        correct = studentPosition == goldPosition
        if not correct:
            self.addMessage('Student answer: ' + str(studentPosition))
            self.addMessage('Correct answer: ' + str(goldPosition))
        return self.testPass(grades) if correct else self.testFail(grades)

    def writeSolution(self, moduleDict, filePath):
        bayesAgentsModule = moduleDict['bayesAgents']
        staffBayesNet, _ = bayesAgentsModule.constructBayesNet(self.startState)
        FOOD_HOUSE_VAR = bayesAgentsModule.FOOD_HOUSE_VAR
        bayesAgentsModule.fillCPTs(staffBayesNet, self.startState)
        staffFunction = bayesAgentsModule.getMostLikelyFoodHousePosition
        answer = staffFunction(self.evidence, staffBayesNet, self.eliminationOrder)[FOOD_HOUSE_VAR]
        with open(filePath, 'w') as handle:
            handle.write('# This is the solution file for %s.\n\nanswer: """\n' % self.path)
            handle.write(str(answer))
            handle.write('\n"""\n')
        return True

    def createPublicVersion(self):
        pass

class VPITest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(VPITest, self).__init__(question, testDict)
        self.targetFunction = testDict['function']
        layoutText = testDict['layout']
        self.layoutName = testDict['layoutName']

        lay = layout.Layout([row.strip() for row in layoutText.split('\n')])
        self.startState = hunters.GameState()
        self.startState.initialize(lay, 0)

        self.evidence = eval(testDict['evidence'])
        self.eliminationOrder = eval(testDict['eliminationOrder'])

    def execute(self, grades, moduleDict, solutionDict):
        # load student code and staff code solutions
        bayesAgentsModule = moduleDict['bayesAgents']
        studentAgent = bayesAgentsModule.VPIAgent()
        studentAgent.registerInitialState(self.startState)
        studentAnswer = eval('studentAgent.{}(self.evidence, self.eliminationOrder)'.format(self.targetFunction))
        goldAnswer = eval(solutionDict['answer'])
        if type(studentAnswer) == float:
            correct = closeNums(studentAnswer, goldAnswer)
        else:
            correct = closeNums(studentAnswer[0], goldAnswer[0]) & closeNums(studentAnswer[1], goldAnswer[1])
        if not correct:
            self.addMessage('Student answer differed from solution by at least .0001')
            self.addMessage('Student answer: ' + str(studentAnswer))
            self.addMessage('Correct answer: ' + str(goldAnswer))
        return self.testPass(grades) if correct else self.testFail(grades)

    def writeSolution(self, moduleDict, filePath):
        bayesAgentsModule = moduleDict['bayesAgents']
        agent = bayesAgentsModule.VPIAgent()
        agent.registerInitialState(self.startState)
        answer = eval('agent.{}(self.evidence, self.eliminationOrder)'.format(self.targetFunction))
        with open(filePath, 'w') as handle:
            handle.write('# This is the solution file for %s.\n\nanswer: """\n' % self.path)
            handle.write(str(answer))
            handle.write('\n"""\n')
        return True

    def createPublicVersion(self):
        pass

def closeNums(x, y):
    return abs(x - y) < 1e-4

def parseFactorInputProblem(testDict, goingToEliminate=False, goingToEvidenceAssign=False):
    parseDict = {}
    variableDomainsDict = {}
    for line in testDict['variableDomainsDict'].split('\n'):
        variable, domain = line.split(' : ')
        variableDomainsDict[variable] = domain.split(' ')

    parseDict['variableDomainsDict'] = variableDomainsDict


    factorsDict = {} # assume args is a list of factor names and maybe a variable name at the end
    if goingToEliminate:
        eliminateVariable = testDict["eliminateVariable"]
        parseDict['eliminateVariable'] = eliminateVariable

    # for normalize need evidence so that normalize is nontrivial
    if goingToEvidenceAssign:
        evidenceAssignmentString = testDict["evidenceDict"]
        evidenceDict = {}
        for line in evidenceAssignmentString.split('\n'):
            if(line.count(' : ')): #so we can pass empty dicts for unnormalized variables
                evidenceVariable, evidenceAssignment = line.split(' : ')
                evidenceDict[evidenceVariable] = evidenceAssignment
        parseDict['evidenceDict'] = evidenceDict

    for factorName in testDict["factorArgs"].split(' '):
        # construct a dict from names to factors and 
        # load a factor from the test file for each

        currentFactor = parseFactorFromFileDict(testDict, variableDomainsDict=variableDomainsDict,
                                                prefix=factorName)
        factorsDict[factorName] = currentFactor

    parseDict['factorsDict'] = factorsDict

    return parseDict

def replaceTestFile(file_path, typeOfTest, inputToTest):
    #Create temp file
    fh, abs_path = mkstemp()
    with open(abs_path,'w') as new_file:
        with open(file_path) as old_file:
            # Assumes that variableDomainsDict is the last 
            # entry in the test file before the factors start to 
            # get enumerated
            for line in old_file:
                new_file.write(line)
                if 'endOfNonFactors' in line:
                    break
        if typeOfTest == 'BayesNet':
            new_file.write("\n" + inputToTest.easierToParseString())
        elif typeOfTest == 'Factors':
            new_file.write("\n" + "\n".join([factor.easierToParseString(prefix=name, 
                                      printVariableDomainsDict=False) for 
                                      name, factor in inputToTest.items()]))


    close(fh)
    #Remove original file
    remove(file_path)
    #Move new file
    move(abs_path, file_path)

def parseFactorFromFileDict(fileDict, variableDomainsDict=None, prefix=None):
    if prefix is None:
        prefix = ''
    if variableDomainsDict is None:
        variableDomainsDict = {}
        for line in fileDict['variableDomainsDict'].split('\n'):
            variable, domain = line.split(' : ')
            variableDomainsDict[variable] = domain.split(' ')
    # construct a dict from names to factors and 
    # load a factor from the test file for each


    unconditionedVariables = []
    for variable in fileDict[prefix + "unconditionedVariables"].split(' '):
        unconditionedVariable = variable.strip()
        unconditionedVariables.append(unconditionedVariable)

    conditionedVariables = []
    for variable in fileDict[prefix + "conditionedVariables"].split(' '):
        conditionedVariable = variable.strip()
        if variable != '':
            conditionedVariables.append(conditionedVariable)

    if 'constructRandomly' not in fileDict or fileDict['constructRandomly'] == 'False':
        currentFactor = bayesNet.Factor(unconditionedVariables, conditionedVariables,
                                        variableDomainsDict)
        for line in fileDict[prefix + 'FactorTable'].split('\n'):
            assignments, probability = line.split(" = ")
            assignmentList = [assignment for assignment in assignments.split(', ')]

            assignmentsDict = {}
            for assignment in assignmentList:
                var, value = assignment.split(' : ')
                assignmentsDict[var] = value
            
            currentFactor.setProbability(assignmentsDict, float(probability))
    elif fileDict['constructRandomly'] == 'True':
        currentFactor = bayesNet.constructAndFillFactorRandomly(unconditionedVariables, conditionedVariables, variableDomainsDict)
    return currentFactor

def parseSolutionBayesNet(solutionDict):
    # needs to be able to parse in a bayes net
    variableDomainsDict = {}
    for line in solutionDict['variableDomainsDict'].split('\n'):
        variable, domain = line.split(' : ')
        variableDomainsDict[variable] = domain.split(' ')

    variables = list(variableDomainsDict.keys())
    edgeList = []
    for variable in variables:
        parents = solutionDict[variable + 'conditionedVariables'].split(' ')
        for parent in parents:
            if parent != '':
                edgeList.append((parent, variable))

    net = bayesNet.constructEmptyBayesNet(variables, edgeList, variableDomainsDict)

    factors = {}
    for variable in variables:
        net.setCPT(variable, parseFactorFromFileDict(solutionDict, variableDomainsDict, variable))

    return net

def parseBayesNetProblem(testDict):
    # needs to be able to parse in a bayes net,
    # and figure out what type of operation to perform and on what
    parseDict = {}

    variableDomainsDict = {}
    for line in testDict['variableDomainsDict'].split('\n'):
        variable, domain = line.split(' : ')
        variableDomainsDict[variable] = domain.split(' ')

    parseDict['variableDomainsDict'] = variableDomainsDict


    
    variables = []
    for line in testDict["variables"].split('\n'):
        
        variable = line.strip()
        variables.append(variable)

    edges = []
    for line in testDict["edges"].split('\n'):
        
        tokens = line.strip().split()
        if len(tokens) == 2:
            edges.append((tokens[0], tokens[1]))

        else:
            raise Exception("[parseBayesNetProblem] Bad evaluation line: |%s|" % (line,))


    # inference query args

    queryVariables = testDict['queryVariables'].split(' ')

    parseDict['queryVariables'] = queryVariables

    evidenceDict = {}
    for line in testDict['evidenceDict'].split('\n'):
        if(line.count(' : ')): #so we can pass empty dicts for unnormalized variables        
            (evidenceVariable, evidenceValue) = line.split(' : ')
            evidenceDict[evidenceVariable] = evidenceValue

    parseDict['evidenceDict'] = evidenceDict

    if testDict['constructRandomly'] == 'False':
        # load from test file
        problemBayesNet = bayesNet.constructEmptyBayesNet(variables, edges, variableDomainsDict)
        for variable in variables:
            currentFactor = bayesNet.Factor([variable], problemBayesNet.inEdges()[variable], variableDomainsDict)
            for line in testDict[variable + 'FactorTable'].split('\n'):
                assignments, probability = line.split(" = ")
                assignmentList = [assignment for assignment in assignments.split(', ')]

                assignmentsDict = {}
                for assignment in assignmentList:
                    var, value = assignment.split(' : ')
                    assignmentsDict[var] = value
                
                currentFactor.setProbability(assignmentsDict, float(probability))
            problemBayesNet.setCPT(variable, currentFactor)
    elif testDict['constructRandomly'] == 'True':
        problemBayesNet = bayesNet.constructRandomlyFilledBayesNet(variables, edges, variableDomainsDict)

    parseDict['problemBayesNet'] = problemBayesNet

    if testDict['alg'] == 'inferenceByVariableElimination':
        variableEliminationOrder = testDict['variableEliminationOrder'].split(' ')
        parseDict['variableEliminationOrder'] = variableEliminationOrder
    elif testDict['alg'] == 'inferenceByLikelihoodWeightingSampling':
        numSamples = int(testDict['numSamples'])
        parseDict['numSamples'] = numSamples

    return parseDict

###################################
####### From fa21 Tracking Project
fixed_order = ['West', 'East', 'Stop', 'South', 'North']

class GameScoreTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(GameScoreTest, self).__init__(question, testDict)
        self.maxMoves = int(self.testDict['maxMoves'])
        self.inference = self.testDict['inference']
        self.layout_str = self.testDict['layout_str'].split('\n')
        self.numRuns = int(self.testDict['numRuns'])
        self.numWinsForCredit = int(self.testDict['numWinsForCredit'])
        self.numGhosts = int(self.testDict['numGhosts'])
        self.layout_name = self.testDict['layout_name']
        self.min_score = int(self.testDict['min_score'])
        self.observe_enable = self.testDict['observe'] == 'True'
        self.elapse_enable = self.testDict['elapse'] == 'True'

    def execute(self, grades, moduleDict, solutionDict):
        ghosts = [SeededRandomGhostAgent(i) for i in range(1,self.numGhosts+1)]
        print(self.inference)
        pac = bustersAgents.GreedyBustersAgent(0, inference = self.inference, ghostAgents = ghosts, observeEnable = self.observe_enable, elapseTimeEnable = self.elapse_enable)
        #if self.inference == "ExactInference":
        #    pac.inferenceModules = [moduleDict['inference'].ExactInference(a) for a in ghosts]
        #else:
        #    print "Error inference type %s -- not implemented" % self.inference
        #    return

        stats = run(self.layout_str, pac, ghosts, self.question.getDisplay(), nGames=self.numRuns, maxMoves=self.maxMoves, quiet = False)
        aboveCount = [s >= self.min_score for s in stats['scores']].count(True)
        msg = "%s) Games won on %s with score above %d: %d/%d" % (self.layout_name, grades.currentQuestion, self.min_score, aboveCount, self.numRuns)
        grades.addMessage(msg)
        if aboveCount >= self.numWinsForCredit:
            grades.assignFullCredit()
            return self.testPass(grades)
        else:
            return self.testFail(grades)

    def writeSolution(self, moduleDict, filePath):
        handle = open(filePath, 'w')
        handle.write('# You must win at least %d/10 games with at least %d points' % (self.numWinsForCredit, self.min_score))
        handle.close()

    def createPublicVersion(self):
        pass

class ZeroWeightTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(ZeroWeightTest, self).__init__(question, testDict)
        self.maxMoves = int(self.testDict['maxMoves'])
        self.inference = self.testDict['inference']
        self.layout_str = self.testDict['layout'].split('\n')
        self.numGhosts = int(self.testDict['numGhosts'])
        self.observe_enable = self.testDict['observe'] == 'True'
        self.elapse_enable = self.testDict['elapse'] == 'True'
        self.ghost = self.testDict['ghost']
        self.seed = int(self.testDict['seed'])

    def execute(self, grades, moduleDict, solutionDict):
        random.seed(self.seed)
        inferenceFunction = getattr(moduleDict['inference'], self.inference)
        ghosts = [globals()[self.ghost](i) for i in range(1, self.numGhosts+1)]
        if self.inference == 'MarginalInference':
            moduleDict['inference'].jointInference = moduleDict['inference'].JointParticleFilter()
        disp = self.question.getDisplay()
        pac = ZeroWeightAgent(inferenceFunction, ghosts, grades, self.seed, disp, elapse=self.elapse_enable, observe=self.observe_enable)
        if self.inference == "ParticleFilter":
            for pfilter in pac.inferenceModules: pfilter.setNumParticles(5000)
        elif self.inference == "MarginalInference":
            moduleDict['inference'].jointInference.setNumParticles(5000)
        run(self.layout_str, pac, ghosts, disp, maxMoves = self.maxMoves)
        if pac.getReset():
            grades.addMessage('%s) successfully handled all weights = 0' % grades.currentQuestion)
            return self.testPass(grades)
        else:
            grades.addMessage('%s) error handling all weights = 0' % grades.currentQuestion)
            return self.testFail(grades)

    def writeSolution(self, moduleDict, filePath):
        handle = open(filePath, 'w')
        handle.write('# This test checks that you successfully handle the case when all particle weights are set to 0\n')
        handle.close()

    def createPublicVersion(self):
        self.testDict['seed'] = '188'
        self.seed = 188

class DoubleInferenceAgentTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(DoubleInferenceAgentTest, self).__init__(question, testDict)
        self.seed = int(self.testDict['seed'])
        self.layout_str = self.testDict['layout'].split('\n')
        self.observe = (self.testDict['observe'] == "True")
        self.elapse = (self.testDict['elapse'] == "True")
        self.checkUniform = (self.testDict['checkUniform'] == 'True')
        self.maxMoves = int(self.testDict['maxMoves'])
        self.numGhosts = int(self.testDict['numGhosts'])
        self.inference = self.testDict['inference']
        self.errorMsg = self.testDict['errorMsg']
        self.L2Tolerance = float(self.testDict['L2Tolerance'])
        self.ghost = self.testDict['ghost']

    def execute(self, grades, moduleDict, solutionDict):
        random.seed(self.seed)
        lines = solutionDict['correctActions'].split('\n')
        moves = []
        # Collect solutions
        for l in lines:
            m = re.match(r'(\d+) (\w+) (.*)', l)
            moves.append((m.group(1), m.group(2), eval(m.group(3))))

        inferenceFunction = getattr(moduleDict['inference'], self.inference)

        ghosts = [globals()[self.ghost](i) for i in range(1, self.numGhosts+1)]
        if self.inference == 'MarginalInference':
            moduleDict['inference'].jointInference = moduleDict['inference'].JointParticleFilter()

        disp = self.question.getDisplay()
        pac = DoubleInferenceAgent(inferenceFunction, moves, ghosts, grades, self.seed, disp, self.inference, elapse=self.elapse,
                observe=self.observe, L2Tolerance=self.L2Tolerance, checkUniform = self.checkUniform)
        if self.inference == "ParticleFilter":
            for pfilter in pac.inferenceModules: pfilter.setNumParticles(5000)
        elif self.inference == "MarginalInference":
            moduleDict['inference'].jointInference.setNumParticles(5000)
        run(self.layout_str, pac, ghosts, disp, maxMoves=self.maxMoves)
        msg = self.errorMsg % pac.errors
        grades.addMessage(("%s) " % (grades.currentQuestion))+msg)
        if pac.errors == 0:
            grades.addPoints(2)
            return self.testPass(grades)
        else:
            return self.testFail(grades)

    def writeSolution(self, moduleDict, filePath):
        random.seed(self.seed)
        if self.inference == 'ParticleFilter':
            self.inference = 'ExactInference'  # use exact inference to generate solution
        inferenceFunction = getattr(moduleDict['inference'], self.inference)

        ghosts = [globals()[self.ghost](i) for i in range(1, self.numGhosts+1)]
        if self.inference == 'MarginalInference':
            moduleDict['inference'].jointInference = moduleDict['inference'].JointParticleFilter()
            moduleDict['inference'].jointInference.setNumParticles(5000)

        pac = InferenceAgent(inferenceFunction, ghosts, self.seed, elapse=self.elapse, observe=self.observe)
        run(self.layout_str, pac, ghosts, self.question.getDisplay(), maxMoves=self.maxMoves)
        # run our gold code here and then write it to a solution file
        answerList = pac.answerList
        handle = open(filePath, 'w')
        handle.write('# move_number action likelihood_dictionary\n')
        handle.write('correctActions: """\n')
        for (moveNum, move, dists) in answerList:
            handle.write('%s %s [' % (moveNum, move))
            for dist in dists:
                handle.write('{')
                for key in dist:
                    handle.write('%s: %s, ' % (key, dist[key]))
                handle.write('}, ')
            handle.write(']\n')
        handle.write('"""\n')
        handle.close()

    def createPublicVersion(self):
        self.testDict['seed'] = '188'
        self.seed = 188

class OutputTest(testClasses.TestCase):

    def __init__(self, question, testDict):
        super(OutputTest, self).__init__(question, testDict)
        self.preamble = compile(testDict.get('preamble', ""), "%s.preamble" % self.getPath(), 'exec')
        self.test = compile(testDict['test'], "%s.test" % self.getPath(), 'eval')
        self.success = testDict['success']
        self.failure = testDict['failure']

    def evalCode(self, moduleDict):
        bindings = dict(moduleDict)
        exec(self.preamble, bindings)
        return eval(self.test, bindings)

    def execute(self, grades, moduleDict, solutionDict):
        result = self.evalCode(moduleDict)
        result = list(map(lambda x: str(x), result))
        result = ' '.join(result)

        if result == solutionDict['result']:
            grades.addMessage('PASS: %s' % self.path)
            grades.addMessage('\t%s' % self.success)
            return True
        else:
            grades.addMessage('FAIL: %s' % self.path)
            grades.addMessage('\t%s' % self.failure)
            grades.addMessage('\tstudent result: "%s"' % result)
            grades.addMessage('\tcorrect result: "%s"' % solutionDict['result'])

        return False

    def writeSolution(self, moduleDict, filePath):
        handle = open(filePath, 'w')
        handle.write('# This is the solution file for %s.\n' % self.path)
        handle.write('# The result of evaluating the test must equal the below when cast to a string.\n')
        solution = self.evalCode(moduleDict)
        solution = list(map(lambda x: str(x), solution))
        handle.write('result: "%s"\n' % ' '.join(solution))
        handle.close()
        return True

    def createPublicVersion(self):
        pass

def run(layout_str, pac, ghosts, disp, nGames = 1, name = 'games', maxMoves=-1, quiet = True):
    "Runs a few games and outputs their statistics."

    starttime = time.time()
    lay = layout.Layout(layout_str)

    #print '*** Running %s on' % name, layname,'%d time(s).' % nGames
    games = busters.runGames(lay, pac, ghosts, disp, nGames, maxMoves)

    #print '*** Finished running %s on' % name, layname, 'after %d seconds.' % (time.time() - starttime)

    stats = {'time': time.time() - starttime, \
      'wins': [g.state.isWin() for g in games].count(True), \
      'games': games, 'scores': [g.state.getScore() for g in games]}
    statTuple = (stats['wins'], len(games), sum(stats['scores']) * 1.0 / len(games))
    if not quiet:
        print('*** Won %d out of %d games. Average score: %f ***' % statTuple)
    return stats


