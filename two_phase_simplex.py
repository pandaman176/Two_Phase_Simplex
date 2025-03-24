import numpy as np
from enum import Enum

class ProblemStatus(Enum):
    UNBOUNDED = 1
    FEASIBLE = 2
    UNFEASIBLE = 3
    UNSOLVED = 4
    SOLVED = 5

class TwoPhaseSimplex:
    def __init__(self, c, A, b):
        self.c = c
        self.A = A
        self.b = b
        self.m = len(A)
        self.n = len(c)
        self.problem_status = ProblemStatus.UNSOLVED

    def _update(self, tableau, basis, pivot_row, pivot_column, pivot):
        """
        Update the tableau using the pivot element.
        """
        #update tableau
        tableau[pivot_row] = tableau[pivot_row] / pivot
        for i in range(self.m + 1):
            if i != pivot_row:
                multiplier = tableau[i][pivot_column]
                tableau[i] = tableau[i] - multiplier * tableau[pivot_row]
        print(tableau)
        #update basis
        print("column", basis[pivot_row - 1], "leave")
        print("column",pivot_column + 1, "enter")
        basis[pivot_row - 1] = pivot_column + 1
        print("current basis", basis)

    def _construct_phase1_tableau(self):
        """
        Construct the phase 1 tableau for the auxiliary problem.
        """
        # Add slack variables to the constraints
        tableau = np.hstack((A, np.eye(self.m)))
        tableau = np.hstack((tableau, b.reshape(-1, 1)))
        top_row = -tableau.sum(axis=0)
        tableau = np.vstack((top_row, tableau))
        for i in range(self.n + self.m):
            if self.n <= i < self.n + self.m:
                tableau[0, i] = 0
        
        return tableau, np.arange(self.n+1, self.n + self.m + 1)
        
    def _solve_phase1(self):
        """
        Solve the phase 1 problem to find an initial basic feasible solution.
        """
        tableau, basis = self._construct_phase1_tableau()
        print("=======Phase 1 initial state=======")
        print(tableau)
        print("current basis", basis)
        print("=====================================")

        while self.problem_status == ProblemStatus.UNSOLVED:
            #find column to come in
            for index, reduce_cost in enumerate(tableau[0]):
                if reduce_cost < 0:
                    pivot_column = index 
                    pivot_row = None
                    pivot = None
                    min_ratio = np.inf
                    for j in range(1, self.m + 1):
                        if tableau[j][pivot_column] > 0:
                            ratio = tableau[j][-1] / tableau[j][pivot_column]
                            if ratio < min_ratio:
                                min_ratio = ratio
                                pivot_row = j
                                pivot = tableau[j][pivot_column]
                    self._update(tableau, basis, pivot_row, pivot_column, pivot)
                    #judge whether to stop
                    if np.all(tableau[0] >= 0):
                        if tableau[0][-1] != 0:
                            self.problem_status = ProblemStatus.UNFEASIBLE
                        else:
                            self.problem_status = ProblemStatus.FEASIBLE
                    print("=====================================")
                    break
        if self.problem_status == ProblemStatus.FEASIBLE:
            print("original problem is feasible")
        elif self.problem_status == ProblemStatus.UNFEASIBLE:
            print("original problem is unfeasible")
        else:
            print("original problem is unbounded")
        print("=======Phase 1 end=======")
        
        return tableau, basis
    
    def _construct_phase2_tableau(self, tableau, basis):
        """
        Construct the phase 2 tableau using the initial basic feasible solution found in phase 1.
        """
        print("====remove auxiliuary indeces from basis====")
        # check if the basis includes any artificial variables
        while np.any(basis > self.n):
            # find the first artificial variable in the basis
            pivot_row = None
            for index, value in enumerate(basis):
                if value > self.n:
                    pivot_row = index + 1
            pivot_column = None
            # pivot_column is the smallest(follow bland's rule by convension) column of original problem but not in basis
            for i in range(1,self.m+1):
                if i not in basis:
                    # pivot cannot be zero which means independednt column
                    if (tableau[pivot_row][i-1] != 0):
                        pivot_column = i-1
                        break

            pivot = tableau[pivot_row][pivot_column]

            self._update(tableau, basis, pivot_row, pivot_column, pivot)
            print("=====================================")
        print("====remove auxiliuary columns====")
        delete_cols = np.arange(self.n, self.n+self.m)
        tableau = np.delete(tableau, delete_cols, axis=1)
        print(tableau)
        print("=====================================")

        # update reducecost and objective value
        print("====update objective value and reduce cost====")
        x = np.zeros(self.n)
        for i in range(self.n):
            if i+1 in basis:
                indices = np.where(basis == i+1)
                x[i] = tableau[indices[0][0]+1][-1]
        A = np.delete(tableau, [0], axis=0)
        A = np.delete(A, [-1], axis=1)
        Xb = np.array([])
        for index, value in enumerate(basis):
            Xb = np.append(Xb, c[value-1])
        new_reduce_costs = c - Xb@A
        new_top_row = np.append(new_reduce_costs, -np.dot(x,c))
        tableau[0] = new_top_row
        print(tableau)
        print("=====================================")
            
        return tableau, basis
    
    def _solve_phase2(self, tableau, basis):
        """
        Solve the phase 2 problem using the initial basic feasible solution found in phase 1.
        """
        tableau, basis = self._construct_phase2_tableau(tableau, basis)
        print("=======Phase 2 initial state=======")
        print(tableau)
        print("basis",basis)
        for index in basis:
            if(tableau[0][index-1]!=0):
                print("\033[31mAttention!\033[0m reduce cost of basis column", index, "is not zero!\nSomething is wrong!")
        print("=====================================")

        while self.problem_status == ProblemStatus.FEASIBLE:
            #find column to come in
            for index, reduce_cost in enumerate(tableau[0]):
                if reduce_cost < 0:
                    pivot_column = index 
                    pivot_row = None
                    pivot = None
                    min_ratio = np.inf
                    for j in range(1, self.m + 1):
                        if tableau[j][pivot_column] > 0:
                            ratio = tableau[j][-1] / tableau[j][pivot_column]
                            if ratio < min_ratio:
                                min_ratio = ratio
                                pivot_row = j
                                pivot = tableau[j][pivot_column]
                    if pivot_row is None:
                        self.problem_status = ProblemStatus.UNBOUNDED
                    
                    self._update(tableau, basis, pivot_row, pivot_column, pivot)
                    #judge whether to stop
                    print("=====================================")
                    if np.all(tableau[0] >= 0):
                        self.problem_status = ProblemStatus.SOLVED
                        print(tableau)
                        print("basis", basis)
                        print("Optimal value:", -tableau[0][-1])
                        x = np.zeros(self.n)
                        for i in range(self.n):
                            if i+1 in basis:
                                indices = np.where(basis == i+1)
                                x[i] = tableau[indices[0][0]+1][-1]
                        print("Optimal Solution:", x)
                        print("=======Phase 2 end=======")
                    break

    def solve(self):
        """
        Solve the linear programming problem.
        """

        if len(c) != A.shape[1]:
            # check c has same length as column number of A
            print("\033[31mError\033[0m: c has different length with column number of A! Did you forget to consider 0 parameter?")
            return
        if np.any(b < 0):
            # check b positive
            print("\033[31mError\033[0m: b has negative elements! Remember to start Simplex method we require b >= 0")
            return

        # Solve the phase 1 problem
        tableau, basis = self._solve_phase1()
        if self.problem_status == ProblemStatus.FEASIBLE:
            self._solve_phase2(tableau, basis)

if __name__ == "__main__":
    """
    This program require you to transform you question to standard form manually.
    e.g.
    minimize: 0x1 + x2 - 4x3 + 0x4 + x5 + 0x6
    subject to:
        x1 + x2 + x3 + x4 = 1
        -x2 - x3 - x4 + x5 = 1
        x1 + 2x3 - x5 - x6 = 0
        x1, x2, x3, x4, x5, x6 >= 0
    """
    c = np.array([3,4,0,0]) 
    #parameter of objective function
    
    A = np.array([
        [1, 1, 1, 0],
        [0,  1, 0,1],
    ])
    # Constarints Matrix

    b = np.array([4,5])
    # right hand side of constraints

    # set precision of print to avoid too many digits
    np.set_printoptions(suppress=True, precision=3)

    simplex = TwoPhaseSimplex(c, A, b)
    simplex.solve()
