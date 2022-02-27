function output = interpolate(inputCoordinate, sampledArray)

    dim = size(sampledArray);
    
    if length(dim) == 2
        
        xi = inputCoordinate(1);
        X = sampledArray(:,1);
        Y = sampledArray(:,2);

        [~, idx] = min(abs(X-xi));

        % initialise values
        dx = xi-X(idx);
        dy = 0;
        Dx = 1e6;

        % if the difference in time is positive, then the output will be
        % after the function value at the nearest sampled timestep,
        % AND within the upper bound of the time vector.
        if (dx > 0) && (idx < length(Y))

            dy = Y(idx+1) - Y(idx);
            Dx = X(idx+1) - X(idx);

        elseif (dx < 0) && (idx > 1)
            
            dy = Y(idx) - Y(idx-1);
            Dx = X(idx) - X(idx-1);
            
        end

        output = Y(idx) + (dy/Dx)*dx;
        
        
    elseif length(dim) == 3
        
        xi = inputCoordinate(1);
        yi = inputCoordinate(2);
        
        X = sampledArray(:,:,1);
        Y = sampledArray(:,:,2);
        Z = sampledArray(:,:,3);
        
        [~, Xidx] = min(abs(X-xi));
        [~, Yidx] = min(abs(Y-yi));
        
        % initialise values
        dx = xi-X(Xidx);
        dy = yi-Y(Yidx);
        dz = 0;
        Dx = 1e6;
        Dy = 1e6;
        
        % if the difference in time is positive, then the output will be
        % after the function value at the nearest sampled timestep,
        % AND within the upper bound of the time vector.
        if (dy > 0) && (Yidx < length(Z))
            
            if (dx > 0) && (Xidx < length(Z))

                dz = Z(Xidx+1, Yidx+1) - Z(Xidx, Yidx);
                Dx = X(Xidx+1) - X(Xidx);
                Dy = Y(Yidx+1) - Y(Yidx);

            elseif (dx < 0) && (Xidx > 1)

                dz = Z(Xidx, Yidx+1) - Z(Xidx-1, Yidx);
                Dx = X(Xidx) - X(Xidx-1);
                Dy = Y(Yidx+1) - Y(Yidx);

            end

        elseif (dy < 0) && (Yidx > 1)
            
            if (dx > 0) && (Xidx < length(Y))

                dz = Z(Xidx+1, Yidx) - Z(Xidx, Yidx-1);
                Dx = X(Xidx+1) - X(Xidx);
                Dy = Y(Yidx) - Y(Yidx-1);

            elseif (dx < 0) && (Xidx > 1)

                dz = Z(Xidx, Yidx) - Z(Xidx-1, Yidx-1);
                Dx = X(Xidx) - X(Xidx-1);
                Dy = Y(Yidx) - Y(Yidx-1);

            end
            
        end

        output = Z(Xidx) + (dz/Dx)*dx + (dz/Dy)*dy;
        
    end
    
    
end
