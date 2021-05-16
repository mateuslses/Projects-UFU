clear all;
close all;
clc;

% Lendo imagem à ser comprimida:
imagem = imread('teste.JPG');
figure,imshow(imagem)

% Convertendo imagem em escala de cinza (diminui variaveis necessarias para calculo de probabilidade):
I = rgb2gray(imagem);

% Tamanho da imgagem:
[m,n] = size(I);
Totalcount = m*n;

% Variaveis para calculo de probabilidade:
cnt = 1;
sigma = 0;

% Cálculo da probabilidade:
for i=0:255
k=I==i;
count(cnt) = sum(k(:))

% Array utilizado para acumular probabilidades:
pro(cnt) = count(cnt)/Totalcount;
sigma = sigma + pro(cnt);
cumpro(cnt) = sigma;
cnt = cnt + 1;
end;

% Simbolos por imagem:
simb = [0:255];

% Criando um dicionario:
dict = huffmandict(simb,pro);

% Convertendo array em vetor:
vec_size = 1;
for p = 1:m
for q = 1:n
newvec(vec_size)=I(p,q);
vec_size = vec_size+1;
end
end

% Função para Codificação de Huffman:
hcode = huffmanenco(newvec,dict);

% Função para Decodificação de Huffman:
dhsig1 = huffmandeco(hcode,dict);

% Convertendo dhsig1 double para matrix de inteiros de 8 bits:
dhsig = uint8(dhsig1);

% Convertendo vetor em array:
dec_row=sqrt(length(dhsig));
dec_col=dec_row;

arr_row = 1;
arr_col = 1;
vec_si = 1;

for x = 1:m
for y = 1:n
back(x,y) = dhsig(vec_si);
arr_col = arr_col+1;
vec_si = vec_si + 1;
end
arr_row = arr_row+1;
end




%Convertendo grayscale para RBG:
[deco, map] = gray2ind(back,256);
RGB = ind2rgb(deco,map);
imwrite(RGB,'deco_teste.JPG');
imshow(RGB);

%Entropia
J = entropy(I)
fprintf('\nEntropia: %f ', J);

%Relacao sinal ruido de pico e o erro quadrático médio:
InputImage=imread('teste.JPG');
ReconstructedImage=imread('deco_teste.JPG');
n=size(InputImage);
M1=n(1);
N1=n(2);
MSE = sum(sum((InputImage-ReconstructedImage).^2))/(M1*N1);
PSNR = 10*log10(256*256/MSE);
fprintf('\nMSE: %f ', MSE);
fprintf('\nPSNR: %f dB', PSNR);

% CR
% Taxa de Compressao = (numero de bits ao final) / (numero de bits originalmente)
