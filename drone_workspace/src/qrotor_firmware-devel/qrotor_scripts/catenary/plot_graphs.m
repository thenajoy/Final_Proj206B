


a = 0.25;

x = -2: 0.01:2;
cat_y = a*0.5*(exp(x)+exp(-x))-a;


k = 0:0.1:1;

paray = zeros(length(k), length(x));
for i = 1:length(k)
    paray(i,:) = k(i)*x.^2;
end
    
%%
figure; hold on;
plot(x,y, 'r', 'linewidth', 3);
for i = 1:length(k)
   plot(x, paray(i,:)); 
end

grid on; grid minor;






