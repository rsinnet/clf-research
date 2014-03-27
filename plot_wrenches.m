function plot_wrenches(hs)

if nargin < 1
    hsx = evalin('base', 'hs');
end

% Start on domain 1
i = 0;
while true
    i = i + 1;
    if hsx.model.getDomain(hsx.solution(i).cons).n == 1
        break
    end
end

n = hsx.model.nExtDof;
nr = hsx.model.nRobotDof;
nb = hsx.model.nBaseDof;
nd = 4;

T1 = cat(2, hsx.solution(i:i-1+nd).x);
T2 = cat(2, hsx.solution((i+nd):(i-1+2*nd)).x);
T = cat(2, T1, T2);

X1 = cat(2, hsx.solution(i:(i-1+nd)).y);
X2 = cat(2, hsx.solution((i+nd):(i-1+2*nd)).y);
X = cat(2, X1, X2);

Qr1 = 180/pi*X1(nb+1:n, :);
Qr2 = 180/pi*X2(nb+1:n, :);

dQr1 = 180/pi*X1(n+nb:2*n, :);
dQr2 = 180/pi*X2(n+nb:2*n, :);

Q = X(1:n,:);
dQ = X(n+1:2*n, :);

Qr = cat(2, Qr1, Qr2);
dQr = cat(2, dQr1, dQr2);

U1 = cat(2, hsx.solution(i:(i-1+nd)).u);
U2 = cat(2, hsx.solution((i+nd):(i-1+2*nd)).u);

Ur1 = cat(2, U1(nb+1:n, :));
Ur2 = cat(2, U2(nb+1:n, :));

Ur = cat(2, Ur1, Ur2);

N = length(T);
ns = hsx.model.nSpatialDim;

hnst = nan(size(T));

% Loop vars:
l = 0;
cpIndexPre = nan(1, 2*nd);
cpIndexPost = nan(1, 2*nd);

if ns == 2
    Ff = nan(9, N);
elseif ns == 3
    Ff = nan(16, N);
end

for j = 0:(2*nd-1)
    d = hsx.model.getDomain(hsx.solution(i+j).cons).n;
    cpIndexPost(j+1) = l + 1;
    for k = 1:size(hsx.solution(i+j).x, 2)
        l = l + 1;
        Ff(:, l) = map_force_to_full(hsx.solution(i+j).F(:, k), d, ...
                                     ns);
        hnst(l) = HeightNST(hsx.solution(i+j).y(1:n, k), ...
                            hsx.solution(i+j).leg);
    end
    cpIndexPre(j+1) = l;
end

%% autofigs
xl = T([1 end]) + .02*[-1, 1]*(T(end) - T(1));
hHeight = 85;
hWidth = 170;
for j = setdiff(1:nr, [2])
    yl = [min(Qr(j,:)), max(Qr(j,:))] + ...
         .05*[-1, 1]*(max(Qr(j, :)) - min(Qr(j, :)));

    h = figure(40+j);
    set(h, 'Name', sprintf('Joint Position q%d', j), ...
           'Position', ...
           [70+hWidth*(j>nr/2), ...
            (nr/2-1)*(hHeight+88)+5 - (hHeight+88)*mod(j-1, nr/2), ...
            hWidth, ...
            hHeight]);
    ax = gca;
    plot(T1, Qr1(j,:), ...
         T2, Qr2(j,:), ...
         T(cpIndexPre), Qr(j, cpIndexPre), 'o', ...
         'LineWidth', 1, ...
         'MarkerSize', 3);
    set(ax, 'XLim', xl, ...
            'YLim', yl, ...
            'FontSize', 7);
    
    ylabel(sprintf('$q_{%d}$ (deg)', j), ...
           'Interpreter', 'LaTeX', ...
           'FontSize', 8);
    grid on
    xL = xlabel('time (s)', ...
                'Interpreter', 'LaTeX', ...
                'FontSize', 8);
    if exist('_fig') ~= 7
        mkdir('_fig')
    end
    outputFile = fullfile(getenv('HOME'), ...
                          'proj', 'Dropbox', ...
                          'IFAC_Automatica_BipedSurveyPaper', ...
                          'figures', ...
                          sprintf('reduction_joint_q%d.eps', j));
    set(h,'PaperPositionMode','auto')
    set(ax, 'Position', [0.23, 0.3, .74, .65])
    
    ydiff = yl(2) - yl(1);
    xLpos = get(xL, 'Position');
    xLpos(2) = yl(1) - .24 * ydiff;
    set(xL, 'Position', xLpos);
    
    print(sprintf('-f%d', h), '-depsc',  outputFile)
end

%% autofigs - actuators
xl = T([1 end]) + .02*[-1, 1]*(T(end) - T(1));
hHeight = 85;
hWidth = 170;
for j = setdiff(1:nr, [2])
    yl = [min(Ur(j,:)), max(Ur(j,:))] + ...
         .05*[-1, 1]*(max(Ur(j, :)) - min(Ur(j, :)));

    h = figure(50+j);
    set(h, 'Name', sprintf('Actuator Torque on q%d', j), ...
           'Position', ...
           [70+hWidth*(j>nr/2), ...
            (nr/2-1)*(hHeight+88)+530 - (hHeight+88)*mod(j-1, nr/2), ...
            hWidth, ...
            hHeight]);
    ax = gca;
    plot(T1, Ur1(j,:), ...
         T2, Ur2(j,:), ...
         T(cpIndexPost), Ur(j, cpIndexPost), 'o', ...
         'LineWidth', 1, ...
         'MarkerSize', 3);
    set(ax, 'XLim', xl, ...
            'YLim', yl, ...
            'FontSize', 7);
    
    ylabel(sprintf('$u_{%d}$ (N$\\cdot$m)', j), ...
           'Interpreter', 'LaTeX', ...
           'FontSize', 8);
    grid on
    xL = xlabel('time (s)', ...
                'Interpreter', 'LaTeX', ...
                'FontSize', 8);
    if exist('_fig') ~= 7
        mkdir('_fig')
    end
    outputFile = fullfile(getenv('HOME'), ...
                          'proj', 'Dropbox', ...
                          'IFAC_Automatica_BipedSurveyPaper', ...
                          'figures', ...
                          sprintf('reduction_torque_u%d.eps', j));
    set(h,'PaperPositionMode','auto')
    set(ax, 'Position', [0.23, 0.3, .74, .65])
    
    ydiff = yl(2) - yl(1);
    xLpos = get(xL, 'Position');
    xLpos(2) = yl(1) - .24 * ydiff;
    set(xL, 'Position', xLpos);
    
    print(sprintf('-f%d', h), '-depsc',  outputFile)
end


%% oldfigs

%h = figure(34); clf;
%set(h, 'Name', 'Reaction forces');
%plot(hsx.solution(3).x, hsx.solution(3).y(1:n,:))
%legend(gca, ...
%       arrayfun(@(x) sprintf('q%d', x), 1:n, ...
%                'UniformOutput', false), ...
%       'Orientation', 'Vertical', ...
%       'FontSize', 6);

%plot(T, Ff);
%xlabel('Time (s)');
%ylabel('Wrench(N|Nm)');
%title('Reaction forces');

%legend(gca, {'sttx', 'sttz', 'sttwy', 'sthx', 'sthz', 'nstx', 'nstz', ...
%             'stk', 'nsk'});

%h = figure(37);
%set(h, 'Name', 'Swing Toe Height');
%ax = gca;
%distinctColors = distinguishable_colors(n-3);
%set(ax, 'ColorOrder', distinctColors);
%plot(T, hnst * 1000);
%title('Swing Toe Height');
%xlabel('Time (s)');
%ylabel('Height (mm)');


function Ff = map_force_to_full(F, d, ns)

if ns == 2
    %sttx, sttz, sttwy, sthx, sthz, nstx, nstz, stk, nsk
    Ff = nan(9, 1);
    switch d
      case 1
        Ff(4:8) = F;
      case 2
        Ff(1:3) = F(1:3);
        Ff(6:8) = F(4:6);
      case 3
        Ff(1:3) = F(1:3);
        Ff(8) = F(4);
      case 4
        Ff(1:3) = F(1:3);
        Ff(8:9) = F(4:5);
    end
elseif ns == 3
    %sttx, stty, sttz, sttwx, sttwy, sttwz, sthx, sthy, sthz, sthwx,
    %sthwz, nstx, nsty, nstz, stk, nsk, 
    Ff = nan(16, 1);
    switch d
      case 1
        Ff(4:8) = F;
      case 2
        Ff(1:3) = F(1:3);
        Ff(6:8) = F(4:6);
      case 3
        Ff(1:3) = F(1:3);
        Ff(8) = F(4);
      case 4
        Ff(1:3) = F(1:3);
        Ff(9) = F(4:5);
    end
end
