# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent
from pacman import GameState

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState: GameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState: GameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** BEGIN YOUR CODE HERE ***"
        if successorGameState.isWin():
            return 9999999
        
        if successorGameState.isLose():
            return -9999999
        
        succFoods = newFood.asList()
        currFoods = currentGameState.getFood().asList()
        currPellets = currentGameState.getCapsules()
        succPellets = successorGameState.getCapsules()
        eval = successorGameState.getScore() - currentGameState.getScore()

        currGhosts = []
        for ghost in currentGameState.getGhostStates():
            currGhosts.append(manhattanDistance(newPos,ghost.getPosition()))

        succGhosts = []
        for ghost in newGhostStates:
            succGhosts.append(manhattanDistance(newPos,ghost.getPosition()))
        foods = []
        for food in newFood.asList():
            foods.append(manhattanDistance(newPos,food))    

        # Encourage eating food
        if(len(succFoods) < len(currFoods)):
            eval += 200
            
        eval += float(1/min(foods))
        
        # Encourage eating pellets
        if(len(succPellets) < len(currPellets)):
            eval += 100 *len(succPellets)
        
        # Encourage eating pellets
        eval -= len(succFoods)
        
        # discourage stopping
        if action == Directions.STOP:
            eval -= 10

        #Checks if you get closer to ghosts
        #if ghosts scared, add eval. Remove if not scared
        if min(succGhosts) < min(currGhosts):
            if sum(newScaredTimes) > 0:
                eval += 20
            else:
                eval -= 100
        else:
            if sum(newScaredTimes) <= 0:
                eval -= 10
            else:
                eval += 200
        
        return eval
        "*** END YOUR CODE HERE ***"
        # ^^^ you should return something in the above block
        
        # but by default, this is the evaluation function before you put your code in
        return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState: GameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """
    def max_value(self, depth, gameState: GameState, agent):
        "max value for game state"
        v = -float('inf')
        if gameState.isLose() or gameState.isWin() or depth+1 == self.depth:
            return self.evaluationFunction(gameState)
        actions = gameState.getLegalActions(agent)
        for action in actions:
            successor = gameState.generateSuccessor(0,action)
            v = max(v, self.min_value(depth+1,successor, agent+1))
        return v

    def min_value(self, depth, gameState: GameState, agent):
        "min value for game state"
        v = float('inf')
        if gameState.isLose() or gameState.isWin():
            return self.evaluationFunction(gameState)
        actions = gameState.getLegalActions(agent)
        for action in actions:
            successor = gameState.generateSuccessor(agent,action)
            if agent == (gameState.getNumAgents()-1):
                v = min(v, self.max_value(depth, successor, 0))
            else:
                v = min(v, self.min_value(depth, successor, agent+1))
        return v
        

    def getAction(self, gameState: GameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** BEGIN YOUR CODE HERE ***"
        currentEval = -float('inf')
        nextAction = None
        for action in gameState.getLegalActions(0):
            successor = gameState.generateSuccessor(0,action)
            eval = self.min_value(0,successor,1)
            if eval > currentEval:
                nextAction = action
                currentEval = eval
                
        return nextAction
        "*** END YOUR CODE HERE ***"

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """
    def max_value(self, depth, alpha,beta, gameState: GameState, agent):
        "max value for game state"
        v = -float('inf')
        if gameState.isLose() or gameState.isWin() or depth+1 == self.depth:
            return self.evaluationFunction(gameState)
        actions = gameState.getLegalActions(agent)
        a = alpha
        for action in actions:
            successor = gameState.generateSuccessor(0,action)
            v = max(v, self.min_value(depth+1, a, beta, successor, agent+1))
            if v > beta:
                return v
            a = max(a,v)
        return v

    def min_value(self, depth,alpha, beta, gameState: GameState, agent):
        "min value for game state"
        v = float('inf')
        if gameState.isLose() or gameState.isWin():
            return self.evaluationFunction(gameState)
        actions = gameState.getLegalActions(agent)
        b = beta
        for action in actions:
            successor = gameState.generateSuccessor(agent,action)
            if agent == (gameState.getNumAgents()-1):
                v = min(v, self.max_value(depth, alpha, b, successor, 0))
                if v < alpha:
                    return v
                b = min(b,v)
            else:
                v = min(v, self.min_value(depth, alpha, b, successor, agent+1))
                if v < alpha:
                    return v
                b = min(b,v)
        return v
        
    def getAction(self, gameState: GameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** BEGIN YOUR CODE HERE ***"
        currentEval = -float('inf')
        nextAction = None
        alpha = -float('inf')
        beta = float('inf')
        for action in gameState.getLegalActions(0):
            successor = gameState.generateSuccessor(0,action)
            eval = self.min_value(0, alpha, beta, successor, 1)
            if eval > currentEval:
                nextAction = action
                currentEval = eval
            if eval > beta:
                return nextAction
            alpha = max(alpha, eval)
        return nextAction
        "*** END YOUR CODE HERE ***"

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """
   
    def max_value(self, depth, gameState: GameState, agent):
        "max value for game state"
        v = -float('inf')
        if gameState.isLose() or gameState.isWin() or depth+1 == self.depth:
            return self.evaluationFunction(gameState)
        actions = gameState.getLegalActions(agent)
        for action in actions:
            successor = gameState.generateSuccessor(0,action)
            v = max(v, self.exp_value(depth+1,successor, agent+1))
        return v

    def exp_value(self, depth, gameState: GameState, agent):
        "expected value for game state"
        v = 0
        if gameState.isLose() or gameState.isWin():
            return self.evaluationFunction(gameState)
        actions = gameState.getLegalActions(agent)
        p = 1/len(actions)
        for action in actions:
            successor = gameState.generateSuccessor(agent,action)
            if agent == (gameState.getNumAgents()-1):
                v += p*self.max_value(depth, successor, 0)
            else:
                v += p*self.exp_value(depth, successor, agent+1)
        return v
    def getAction(self, gameState: GameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** BEGIN YOUR CODE HERE ***"
        currentEval = -float('inf')
        nextAction = None
        for action in gameState.getLegalActions(0):
            successor = gameState.generateSuccessor(0,action)
            eval = self.exp_value(0,successor,1)
            if eval > currentEval:
                nextAction = action
                currentEval = eval
                
        return nextAction
        "*** END YOUR CODE HERE ***"

def betterEvaluationFunction(currentGameState: GameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** BEGIN YOUR CODE HERE ***"
# Useful information you can extract from a GameState (pacman.py)
    newPos = currentGameState.getPacmanPosition()
    newFood = currentGameState.getFood()
    newGhostStates = currentGameState.getGhostStates()
    newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

    "*** BEGIN YOUR CODE HERE ***"
    if currentGameState.isWin():
        return 9999999
    
    if currentGameState.isLose():
        return -9999999
    
    succFoods = newFood.asList()
    currFoods = currentGameState.getFood().asList()
    currPellets = currentGameState.getCapsules()
    eval = currentGameState.getScore()

    currGhosts = []
    for ghost in currentGameState.getGhostStates():
        currGhosts.append(manhattanDistance(newPos,ghost.getPosition()))

    succGhosts = []
    for ghost in newGhostStates:
        succGhosts.append(manhattanDistance(newPos,ghost.getPosition()))
    foods = []
    for food in newFood.asList():
        foods.append(manhattanDistance(newPos,food))    

    # Encourage eating food
    if(len(succFoods) < len(currFoods)):
        eval += 200
        
    eval += float(1/min(foods))
    
    # Encourage eating pellets
    eval -= len(succFoods)

    #Checks if you get closer to ghosts
    #if ghosts scared, add eval. Remove if not scared
    if min(succGhosts) < min(currGhosts):
        if sum(newScaredTimes) > 0:
            eval += 20
        else:
            eval -= 100
    else:
        if sum(newScaredTimes) <= 0:
            eval -= 10
        else:
            eval += 200
    
    return eval
"*** END YOUR CODE HERE ***"

# Abbreviation
better = betterEvaluationFunction
