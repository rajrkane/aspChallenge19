# aspChallenge19

This is a proposed solution to the [ASP Challenge 2019](https://sites.google.com/view/aspcomp2019/). It models an simplified automated warehouse scenario using Answer Set Programming with Clingo. Given an initial state representing a warehouse, it finds an optimal solution for robots to fulfill orders in the fewest time steps possible.

## usage

Run `clingo solution.asp instances/<instance> -c m=<time>`, substituting the name of the instance file and the integer number of time steps.

## results

| Instance    | Time steps  |
| ----------- | ----------- |
| inst1.asp   | 7           |
| inst2.asp   | 6           |
| inst3.asp   | 5           |
| inst4.asp   | 5           |
| inst5.asp   | 5           |
