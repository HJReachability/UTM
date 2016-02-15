function numSafetyChecks()
% Number of vehicles
Ns = 2:100;

% Number of vehicles per platoon
k = 5;

% Number of platoons
f = 1:-0.2:0;
num_f = length(f);

% Plot
colors = lines(num_f);
hs = [];
h = figure;
for i = 1:num_f;
  checks = numSafetyChecksPlatoon(Ns, f(i), k);
  hs = [hs plot(Ns, checks, 'color', colors(i,:), 'linewidth', 2)];
  hold on
end

title({'Numer of safety checks for various fractions of', ['vehicles in ' ...
  num2str(k) '-vehicle platoons']}, 'fontsize', 14)
lgd_text = cellstr( num2str(f') );
for i = 1:length(lgd_text)
  lgd_text{i} = ['frac. of veh. in platoons: ' lgd_text{i}];
end
legend(hs, lgd_text, 'location', 'northwest','fontsize', 14)
xlabel('Total number of vehicles','fontsize', 14)
ylabel('Number of safety checks', 'fontsize', 14)

h.Children(2).FontSize = 14;
h.Position(1:2) = [200 200];
h.Position(3:4) = [800 600];
end

function checks = numSafetyChecksPlatoon(Ns, f, k)
% Ns - total number of vehicles (vector)
% p - number of platoons (vector)
% k - number of vehicles in each platoon (scalar)
% f - fraction of vehicles in platoons (scalar)

% number of free and platoon vehicles
num_in_p = ceil(f*Ns);
p = ceil(num_in_p/k);
num_free = Ns - num_in_p;

if p > 0
  % number of checks for each group
  chks_in_full_p = (p-1) * (k-1);
  chks_in_last_p = mod(p, k) - 1;
  chks_in_free = num_free .* (num_free - 1)/2;
  
  checks = chks_in_full_p + chks_in_last_p + chks_in_free;
else
  checks = num_free .* (num_free - 1) /2;
end
end