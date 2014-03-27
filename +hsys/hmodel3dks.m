%HMODEL3DKS   Same as HSYS.HMODEL3Dflat, but does fsolve about knee-strike
classdef hmodel3dks < hsys.hmodel3dflat
    methods
        function this = hmodel3dks()
        % Default constructor
            this = this@hsys.hmodel3dflat();
        end

        % Solves for the missing coordinates necessary to place
        % the system on the guard.
        function qi = solve_qi(this, xr, leg)
            qi = nan;
        end

        function fval = guardSolve(this, xr, qi, leg)
            fval = nan;
        end

        % x_iota provides an embedding from fsolve coordinates to
        % the generalized coordinate space.
        function x = x_iota(this, x, qi, leg)
        % Embed from restricted guard space to full coordinates.
            nb = this.nBaseDof;

            qf  = [zeros(nb,1); x(1); x(2); 0; x(3);
                   x(4);  0;  x(5); x(6)];
            dqf = [zeros(nb,1); x(7);  x(8); 0; x(9);
                   x(10); x(11); x(12); x(13)];

            x = [qf; dqf];
        end

        % x_pi is the canonical projection associated with x_iota
        % and as such, like x_iota, it only applies if the system
        % is on the guard.
        function x = x_pi(this, x)
        % We basically just remove the extra coordinates which
        % should be zeros or very close.
            [q, dq] = this.splitState(x);
            % Well will extract only the necessary coordinates at knee-strike
            x = [q([this.COORDS_QX_STA; this.COORDS_QY_STA
                    this.COORDS_QY_STH; this.COORDS_QY_NSH;
                    this.COORDS_QY_NSA; this.COORDS_QX_NSA]);
                 dq([this.COORDS_QX_STA; this.COORDS_QY_STA;
                     this.COORDS_QY_STH; this.COORDS_QY_NSH;
                     this.COORDS_QY_NSK; this.COORDS_QY_NSA;
                     this.COORDS_QX_NSA])];
        end
    end
end
