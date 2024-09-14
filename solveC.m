function point = solveC(B, E, L1, L4)
    % Define the symbolic variables
    syms x y;

    % Define the equations
    eq1 = (x - B.x)^2 + (y - B.y)^2 == L1.^2;
    eq2 = (x - E.x)^2 + (y - E.y)^2 == L4.^2;

    % Solve the system of equations
    sol = solve([eq1, eq2], [x, y]);

    % Convert the symbolic solutions to numeric values
    x_sol = double(sol.x);
    y_sol = double(sol.y);

       % Check if the solutions are real
    % if ~isreal(x_sol) || ~isreal(y_sol)
    %     error('The solution contains imaginary numbers.');
    % end

    % Check which solution to use based on x-coordinate
    if x_sol(1) > x_sol(2)
        point.x = x_sol(1);
        point.y = y_sol(1);
    else
        point.x = x_sol(2);
        point.y = y_sol(2);
    end
end
