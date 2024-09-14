function E = solveE(D, P, L3)
    % Extract coordinates from struct
    xD = D.x; yD = D.y;
    xP = P.x; yP = P.y;

    % Distance between D and P
    d = sqrt((xD - xP)^2 + (yD - yP)^2);

    % Direction vector from D to P
    direction = [xP - xD, yP - yD];
    % Normalize the direction vector
    direction = direction / norm(direction);

    % Calculate point E by moving L units from D towards P
    E_coords = [xD, yD] + L3 * direction;

    % Ensure E is between D and P
    if norm(E_coords - [xP, yP]) > d || norm(E_coords - [xD, yD]) > d
        error('E is not between D and P with the given length L.');
    end

    % Return point E as a struct
    E.x = E_coords(1);
    E.y = E_coords(2);
end