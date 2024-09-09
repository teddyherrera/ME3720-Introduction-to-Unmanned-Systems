function [orderedHeading] = CTE(bx,by,fx,fy,tx,ty,rho)
% inputs: bx,by     current x/y position
%         fx,fy     "from" point x/y position
%         tx,ty     "to" point x/y position
%         rho       desired approach point (distance in front of vehicle)



% T=[(tx-fx);(ty-fy)];
N = [(ty-fy);-(tx-fx)];
P = [(bx-fx);(by-fy)];
e = P'* N/sqrt(N'*N);
% s1=P'*T/sqrt(T'*T);
psitrack = atan2((ty-fy),(tx-fx));

psicom = psitrack + atan2(e,rho);

orderedHeading = wrapTo180(rad2deg(psicom));

end