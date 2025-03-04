# Two-Phase Simplex Method

This project implements the Two-Phase Simplex Method for solving linear programming problems. This program simulates the manual calculation of the simplex tableau, aiming to help users verify the correctness of their simplex tableau calculations.

## Files

- `two_phase_simplex.py`: Contains the implementation of the Two-Phase Simplex Method.

## Usage

To use this implementation, you need to transform your linear programming problem into standard form manually. The standard form requires all variables to be non-negative and all constraints to be equalities.

### Example

Consider the following linear programming problem:

```
minimize: 0x1 + x2 - 4x3 + 0x4 + x5 + 0x6
subject to:
    x1 + x2 + x3 + x4 = 1
    -x2 - x3 - x4 + x5 = 1
    x1 + 2x3 - x5 - x6 = 0
    x1, x2, x3, x4, x5, x6 >= 0
```

You can solve this problem using the Two-Phase Simplex Method as follows:

1. Define the objective function coefficients `c`:
    ```python
    c = np.array([0, 1, -4, 0, 1, 0])
    ```

2. Define the constraints matrix `A`:
    ```python
    A = np.array([
        [1, 1, 1, 1, 0, 0],
        [0, -1, -1, -1, 1, 0],
        [1, 0, 2, 0, -1, -1]
    ])
    ```

3. Define the right-hand side of the constraints `b`:
    ```python
    b = np.array([1, 1, 0])
    ```

4. Create an instance of the `TwoPhaseSimplex` class and solve the problem:
    ```python
    simplex = TwoPhaseSimplex(c, A, b)
    simplex.solve()
    ```

### Running the Example

To run the example provided in the `two_phase_simplex.py` file, execute the script:
(require numpy)

```bash
python two_phase_simplex.py
```

## Output

The script will print the intermediate steps and the final optimal solution, including the optimal value and the values of the decision variables.

## License

This project is licensed under the MIT License.
