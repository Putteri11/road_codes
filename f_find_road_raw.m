function [ sub_pc, sub_i_profs ] = f_find_road_raw( Xyzti, ins_prof_pc )
%f_find_road_raw very roughly retrieves the road from the point cloud
%Xyzti.
%   This is a very rough function and should only be used for testing purposes.

n_pc = length(Xyzti);
first_prof = ins_prof_pc(1);
n_profs = max(ins_prof_pc) - first_prof + 1;
n_pc_profs = zeros(n_profs, 1);
help_var = first_prof;
for ii=1:n_pc
    if ins_prof_pc(ii) ~= help_var
        help_var = ins_prof_pc(ii);
    end
    n_pc_profs(help_var - first_prof + 1) = n_pc_profs(help_var - first_prof + 1) + 1;
end

n_points = round(mean(n_pc_profs)*0.43);
li = zeros(n_pc, 1);
grad_z_th = 0.01;
n_i_th = 200;
start_i = n_pc_profs(1) + 1;

for i=2:n_profs-1
    end_i = start_i - 1 + n_pc_profs(i);
    grad_z = gradient(Xyzti(start_i:end_i, 3));
    ix = find(abs(grad_z(1:end)) < grad_z_th, 10000);
    ix2 = find(conv(double(diff(ix)==1), ones(1,n_i_th-1), 'valid')==n_i_th-1);
    first_i = ix(ix2(1))+60;
    ix_range = start_i + first_i:start_i + first_i + n_points - 1;
    li(ix_range) = 1;
    start_i = end_i + 1;
end

li = logical(li);
sub_pc = Xyzti(li, :);
sub_i_profs = ins_prof_pc(li);

end

