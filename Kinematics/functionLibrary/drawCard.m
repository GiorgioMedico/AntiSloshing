function drawCard(pos,col)
    


    W1=158;
    H1=293.5;
    W2=344;
    H2=64;
    H3=73;
    plot(pos(1),pos(2),'+','MarkerSize',10,'LineWidth',3,'Color',col);

    
theta=pos(3);

pos=pos';

R2=[cos(theta) -sin(theta);
    sin(theta) cos(theta);];



%% Vertici
LL=pos(1:2)+R2*[-W1/2; 0;]; LR=pos(1:2)+R2*[W1/2;0;];
UL=pos(1:2)+R2*[-W1/2;H1;]; UR=pos(1:2)+R2*[W1/2;H1;];




line([LL(1) LR(1)],[LL(2) LR(2)],'Color',col,'LineWidth',2);
line([LR(1) UR(1)],[LR(2) UR(2)],'Color',col,'LineWidth',2);
line([UR(1) UL(1)],[UR(2) UL(2)],'Color',col,'LineWidth',2);
line([LL(1) UL(1)],[LL(2) UL(2)],'Color',col,'LineWidth',2);

ll=pos(1:2)+R2*[-W2/2; H2+H3;]; lr=pos(1:2)+R2*[+W2/2; H2+H3;];
ul=pos(1:2)+R2*[-W2/2; H2+H3+H2;];    ur=pos(1:2)+R2*[+W2/2; H2+H3+H2;];


line([ll(1) lr(1)],[ll(2) lr(2)],'Color',col,'LineWidth',2);
line([lr(1) ur(1)],[lr(2) ur(2)],'Color',col,'LineWidth',2);
line([ur(1) ul(1)],[ur(2) ul(2)],'Color',col,'LineWidth',2);
line([ll(1) ul(1)],[ll(2) ul(2)],'Color',col,'LineWidth',2);

