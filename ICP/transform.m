%这个函数将icp点云图像转换为txt矩阵形式进行存储
function file = transform(filename)
%filename = '0.ply';
plycloud = pcread(filename);
%pcshow(plycloud)
%获取三维点坐标矩阵
Matrix = double(plycloud.Location());
%创建相应名称txt文件
number = strsplit(filename,'.ply');
txt_filename = strcat(number{1},'.txt');
fd = fopen(txt_filename,'wt');
%写入Data
[row,len]=size(Matrix);
for i = 1:row
    for j = 1:len-1
        fprintf(fd,'%.15f\t',Matrix(i,j));
    end
    fprintf(fd,'%.15f\n',Matrix(i,len));
end
fclose(fd);
 %读入txt文件为mat文件
file = load(txt_filename);