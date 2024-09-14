function point = solveD(A, P, L2, LDP)
    % Define the symbolic variables
    syms x y;

    % Define the equations
    eq1 = (x - A.x)^2 + (y - A.y)^2 == L2.^2;
    eq2 = (x - P.x)^2 + (y - P.y)^2 == LDP.^2;

    % Solve the system of equations
    sol = solve([eq1, eq2], [x, y]);

    % Convert the symbolic solutions to numeric values
    x_sol = double(sol.x);
    y_sol = double(sol.y);

    % Check which solution to use based on x-coordinate
    if x_sol(1) < x_sol(2)
        point.x = x_sol(1);
        point.y = y_sol(1);
    else
        point.x = x_sol(2);
        point.y = y_sol(2);
    end
end
