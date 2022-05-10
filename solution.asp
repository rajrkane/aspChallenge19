%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% File: solution.asp %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% initialize %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% node
node(N) :-
    init(object(node, N), value(at, pair(X, Y))).
nodeAt(N, X, Y) :-
    init(object(node, N), value(at, pair(X, Y))).

% highway
highway(X, Y) :-
    init(object(highway, H), value(at, pair(X, Y))).

% picking station
pickStat(PS, X, Y) :-
    init(object(pickingStation, PS), value(at, pair(X, Y))).

% robot
robot(R) :-
    init(object(robot, R), value(at, pair(X, Y))).
robotAt(R, X, Y, 0) :-
    init(object(robot, R), value(at, pair(X, Y))),
    nodeAt(N, X, Y).

% shelf
shelf(S) :-
    init(object(shelf, S), value(at, pair(X, Y))).
shelfAt(S, X, Y, 0) :-
    init(object(shelf, S), value(at, pair(X, Y))),
    nodeAt(N, X, Y).

% product
product(P) :-
    init(object(product, P), value(on, pair(S, U))).
productOn(P, S, U, 0) :-
    init(object(product, P), value(on, pair(S, U))).

% order
order(O, P, U, 0) :-
    init(object(order, O), value(line, pair(P, U))).
orderPS(O, PS) :-
    init(object(order, O), value(pickingStation, PS)).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% robot constraints %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot on exactly one node
:- 2{robotAt(R, X, Y, T) : nodeAt(N, X, Y)},
    robot(R),
    T = 0..m.

% two robots cannot be on same node
:- 2{robotAt(R, X, Y, T) : robot(R)},
    nodeAt(N, X, Y),
    T = 0..m.

% two robots cannot switch cells
:- robotAt(R1, X1, Y1, T),
    nodeAt(N1, X1, Y1),
    robotAt(R2, X2, Y2, T),
    nodeAt(N2, X2, Y2),
    robotAt(R1, X2, Y2, T+1),
    robotAt(R2, X1, Y1, T+1),
    N1 != N2,
    R1 != R2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% shelf constraints %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% shelf on exactly one node
:- 2{shelfAt(S, X, Y, T) : nodeAt(N, X, Y)},
    shelf(S),
    T = 0..m.

% two shelves cannot be on same node
% so, robot cannot carry two shelves
:- 2{shelfAt(S, X, Y, T) : shelf(S)},
    nodeAt(N, X, Y),
    T = 0..m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% move %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% move directions
move(1,0; -1,0; 0,1; 0,-1).

% destination must be within grid
:- robotAt(R, X, Y, T), not nodeAt(_, X, Y).

% generate movement
{robotMove(R, X, Y, T)} :-
    robot(R),
    move(X, Y),
    T=1..m.
occurs(object(robot, R), move(X, Y), T) :-
    robotMove(R, X, Y, T), T=1..m.

% state change due to move
robotAt(R, X+A, Y+B, T) :-
    robotAt(R, X, Y, T-1),
    robotMove(R, A, B, T),
    T=1..m.
shelfAt(S, X+A, Y+B, T) :-
    robotAt(R, X, Y, T-1),
    carry(R, S, T-1),
    robotMove(R, A, B, T),
    T=1..m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% pickup %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% shelf and robot must be at same location
:- pickup(R, S, T),
    robotAt(R, X1, Y1, T-1),
    nodeAt(N1, X1, Y1),
    shelfAt(S, X2, Y2, T-1),
    nodeAt(N2, X2, Y2),
    N1 != N2.

% robot must not already carry a shelf
:- carry(R, S, T-1),
    pickup(R, S, T),
    T=1..m.

% generate pickup
{pickup(R, S, T) : shelf(S)}1 :-
    robot(R),
    T=1..m.
occurs(object(robot, R), pickup, T) :-
    pickup(R, S, T).

% state change due to pickup
carry(R, S, T) :- pickup(R, S, T).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% putdown %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot must carry some shelf at T
:- putdown(R, T),
    not carry(R, S, T),
    shelf(S),
    T=1..m.

% robot must not be on highway at T
:- putdown(R, T),
    robotAt(R, X, Y, T),
    nodeAt(N, X, Y),
    highway(X, Y),
    T=1..m.

% generate putdown
{putdown(R, T)} :-
    robot(R),
    shelf(S),
    carry(R, S, T-1),
    T=1..m.
occurs(object(robot, R), putdown, T) :-
    putdown(R, T).

% state change due to putdown
not carry(R, S, T) :-
    occurs(object(robot, R), putdown, T),
    carry(R, S, T-1),
    T=1..m.
shelfAt(S, X, Y, T) :-
    carry(R, S, T-1),
    robotAt(R, X, Y, T-1),
    nodeAt(N, X, Y),
    occurs(object(robot, R), putdown, T),
    T=1..m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% deliver %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% amount delivered is positive
:- occurs(object(robot, R),
    deliver(O, P, U), T), U < 1.

% generate delivery
{deliver(R, O, P, U1, T)} :-
    robotAt(R, X, Y, T-1),
    pickStat(PS, X, Y),
    orderPS(O, PS),
    order(O, P, U1, T-1),
    productOn(P, S, U2, T-1),
    U2 >= U1,
    carry(R, S, T-1),
    T=1..m.
occurs(object(robot, R), deliver(O, P, U), T) :-
    deliver(R, O, P, U, T).

% state change due to delivery
% product amount on shelf reduces by amount delivered
productOn(P, S, U1 - U, T) :-
    productOn(P, S, U1, T-1),
    occurs(object(robot, R), deliver(O, P, U), T),
    T=1..m.
% product amount in order reduces by amount delivered
order(O, P, U1 - U, T) :-
    order(O, P, U1, T-1),
    occurs(object(robot, R), deliver(O, P, U), T),
    T=1..m.
% neither reduced quantity may be negative
:- productOn(P, S, U, T), U < 0.
:- order(O, P, U, T), U < 0.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% axioms %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% law of inertia
{robotAt(R, X, Y, T)} :-
    robotAt(R, X, Y, T-1),
    T=1..m.
{carry(R, S, T)} :-
    carry(R, S, T-1),
    T=1..m.
{shelfAt(S, X, Y, T)} :-
    shelfAt(S, X, Y, T-1),
    T=1..m.
{productOn(P, S, U, T)} :-
    productOn(P, S, U, T-1),
    T=1..m.
{order(O, P, U, T)} :-
    order(O, P, U, T-1),
    T=1..m.

% no more than one action per robot per time step
:- occurs(object(robot, R), A, T),
    occurs(object(robot, R), B, T),
    A != B.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% goal %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

:- order(O, P, _, 0),
    not order(O, P, 0, m).
#minimize{T : occurs(_, _, T)}.

#show occurs/3.
