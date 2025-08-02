function best_solution = M_AHA(cost_function, nPop, maxIter, lb, ub)
    % M_AHA: Modified Artificial Hummingbird Algorithm for optimization
    % Inputs:
    %   cost_function : handle to the objective function
    %   nPop          : number of individuals in the population
    %   maxIter       : maximum number of iterations
    %   lb, ub        : lower and upper bounds for parameters
    % Output:
    %   best_solution : best parameters found

    % Number of parameters (dimensions)
    dim = length(lb);

    % Initialize population randomly within bounds
    pop = rand(nPop, dim) .* (ub - lb) + lb;
    
    % Evaluate initial fitness
    fitness = zeros(1, nPop);
    for i = 1:nPop
        fitness(i) = cost_function(pop(i, :));
    end

    % Get best solution
    [best_fitness, best_idx] = min(fitness);
    best_solution = pop(best_idx, :);

    % Main optimization loop
    for iter = 1:maxIter
        % Foraging behavior (exploration)
        new_pop = pop + 0.1 * (rand(nPop, dim) - 0.5) .* (ub - lb);
        
        % Ensure new population is within bounds
        new_pop = max(min(new_pop, ub), lb);
        
        % Evaluate new fitness
        new_fitness = zeros(1, nPop);
        for i = 1:nPop
            new_fitness(i) = cost_function(new_pop(i, :));
        end
        
        % Update solutions if improvement is found
        for i = 1:nPop
            if new_fitness(i) < fitness(i)
                pop(i, :) = new_pop(i, :);
                fitness(i) = new_fitness(i);
            end
        end
        
        % Update best solution
        [best_fitness, best_idx] = min(fitness);
        best_solution = pop(best_idx, :);
        
        % Display progress
        disp(['Iteration ', num2str(iter), ' - Best Cost: ', num2str(best_fitness)]);
    end
end
