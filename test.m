
ticID = tic;
counts = zeros(n_profs, 1);
a = ins_prof_pc(1);

for ii=1:n_pc
    if ins_prof_pc(ii) ~= a
        a = ins_prof_pc(ii);
    end
    counts(a) = counts(a) + 1;
end % constructing counts

plot(counts, 'b.')

t_elapsed_profRetr = toc(ticID);
disp(['Time elapsed: ', num2str(t_elapsed_profRetr), ' seconds.'])

% round(mean(counts))


%%

[li_1, pc_1] = Carve_Several_2D_Objects_edit_Joona(Xyzti(1:10:end, 1:2), 3); % haetaan alueet harvemmasta pilvestä
li_2 = logical(Carve_Several_2D_Objects(Xyzti(:, 1:2), pc_1)); % haetaan alueissa olevat pisteet koko pilvestä

%%
tic
sub_pc = Xyzti(li_2, :);
sub_n_pc = length(sub_pc);
sub_i_profs = ins_prof_pc(li_2);
sub_n_profs = max(sub_i_profs) - sub_i_profs(1) + 1;
sub_counts = zeros(sub_n_profs, 1);
aa = sub_i_profs(1);

for ii=1:sub_n_pc
    if sub_i_profs(ii)==aa
        sub_counts(aa - sub_i_profs(1) + 1) = sub_counts(aa - sub_i_profs(1) + 1) + 1;
    else
        aa = sub_i_profs(ii);
        sub_counts(aa - sub_i_profs(1) + 1) = sub_counts(aa - sub_i_profs(1) + 1) + 1;
    end
end

% monte carlo 
% i_th = 1600 + 50*randn;
% grad_z_th = 0.0004 + 0.0001*randn;
% d_th = 3.0 + 0.2*randn;
% percentage = 0.4 + 0.05*randn;


% fixed values
i_th = 10000;
grad_z_th = 0.005;
d_th = 1.5;

test_li = zeros(sub_n_pc, 1);

for i=2:sub_n_profs-1
    prof_road = sub_pc(logical(i-1+sub_i_profs(1)==sub_i_profs), :);
    grad_z = gradient(prof_road(:, 3));
    l_prof = length(prof_road(:, 1));
    for ii = 2:l_prof-1
        dist1 = sqrt( (prof_road(ii, 1) - prof_road(ii-1, 1))^2 + ...
            (prof_road(ii, 2) - prof_road(ii-1, 2))^2);
        dist2 = sqrt( (prof_road(ii, 1) - prof_road(ii+1, 1))^2 + ...
            (prof_road(ii, 2) - prof_road(ii+1, 2))^2);
        dist_frac = dist1 / dist2;
        isDefect_d = dist_frac > d_th || dist_frac < 1/d_th;
        
        intensity = prof_road(ii, 5);
        isDefect_i = intensity < i_th;
        
        abs_grad_z = abs(grad_z(ii));
        isDefect_grad_z = abs_grad_z > grad_z_th;
        
        isDefect = isDefect_d && isDefect_i && isDefect_grad_z;
        
        if isDefect
            helper = cumsum(sub_counts);
            index = helper(i - 1) + ii;
            test_li(index) = 1;
        end
    end
end


test_li_indices = find(test_li == 1, sub_n_pc);
neighbouring_indices = find(diff(test_li_indices) == 1, length(test_li_indices));

out_li = zeros(sub_n_pc, 1);
out_li(test_li_indices(neighbouring_indices)) = 1;
out_li(test_li_indices(neighbouring_indices) + 1) = 1;

out_li = logical(out_li);

% out_li = logical(test_li);

toc

f_initFig(1, 'w');
% plot(Xyzti(:, 1), Xyzti(:, 2), '.', 'markersize', 2, 'color', 0.7*[1 1 1])
% plot(Xyzti(li_2, 1), Xyzti(li_2, 2), 'o', 'markersize', 6)
fscatter3_edit_Joona(sub_pc(:, 1), sub_pc(:, 2), sub_pc(:, 3), sub_pc(:, 5), cmap);
plot3(sub_pc(out_li, 1), sub_pc(out_li, 2), sub_pc(out_li, 3), 'ro', 'markersize', 6);

out_text = ['i_th: ', num2str(i_th), '; ', '%: ', num2str(percentage), '; ', ...
    'grad_z_th: ', num2str(grad_z_th), '; ', 'd_th: ', num2str(d_th)];

disp(out_text);


%%

f_initFig(2, 'w');
li_3 = logical(sub_i_profs == 16 + sub_i_profs(1) - 1);
fscatter3(sub_pc(li_3, 1), sub_pc(li_3, 2), sub_pc(li_3, 3), sub_pc(li_3, 5), cmap)


%%
ticID = tic;
n_points = round(mean(counts)*0.485);
z_2D = zeros(n_profs-2, n_points);
ix_of_interest = zeros(n_profs, 2);
grad_z_th = 0.01;
n_i_th = 200;
start_i = counts(1) + 1;
dummy = 0;

for i=2:n_profs-1
    end_i = start_i - 1 + counts(i);
    %     try
    z = Xyzti(start_i:end_i, 3);
    grad_z = gradient(Xyzti(start_i:end_i, 3));
    ix = find(abs(grad_z(1:end)) < grad_z_th, 10000);
    ix2 = find(conv(double(diff(ix)==1), ones(1,n_i_th-1), 'valid')==n_i_th-1);
    first_i = ix(ix2(1))+50;
    z_2D(i-1, :) = z(first_i:first_i + n_points - 1);
    ix_of_interest(i, :) = [start_i + first_i, start_i + first_i + n_points - 1];
    dummy = dummy + 1;
    %     catch
    %     end
    start_i = end_i + 1;
end

if dummy == n_profs - 2
    disp('all points gotten');
else
    z_2D = z_2D(1:dummy, :);
    ix_of_interest = ix_of_interest(1:dummy, :);
end

n_skip = 10;

close all;
surf(z_2D(1:n_skip:end, 1:n_skip:end));

t_elapsed_profRetr = toc(ticID);
disp(['Time elapsed: ', num2str(t_elapsed_profRetr), ' seconds.'])


