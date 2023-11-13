classdef Tube < handle

    properties
        id = 0
        od = 0
        r = 0
        k = 0
        l = 0
        d = 0
        E = 0
        I = 0
        J = 0
        G = 0

    end

    methods
        function self = Tube(id, od, r, l, d, E)
            self.id = id;
            self.od = od;
            self.r = r;
            self.k = 1/r;
            self.l = l;
            self.d = d;
            self.E = E;
            self.I = (pi/64)*(od^4 - id^4);
            self.J = 2*self.I;
            self.G = self.E/(2 * (1 + 0.217));

        end

        function params = get_tube_params(self)
            params = [self.id, self.od, self.r, self.k, self.d, self.E, self.I];
        end
    end
end