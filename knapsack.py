import sys
import numpy as np
import gurobipy as gp
from gurobipy import GRB

# Problema da mochila 0-1

# Considere um conjunto com $n$ itens onde cada item $j$ tem um peso $w_j$ e um valor associado $p_j$, e uma mochila com capacidade $c$. O objetivo é colocar esses itens dentro de uma mochila respeitando a capacidade da mochila e obtemos a soma máxima de valores.

# \begin{align*}
# \max \ & \sum_{i=1}^{{n}  p_{i} x_{i} \\
#        & \sum _{i=1}^{n} w_{i} x_{i} \leq c \\
#        & x_{i} \in \{ 0, 1 \} \{ para } i = 1, \ldots, n \\
# \end{align*}

# Para executar o arquivo digite
# python3 qkn_lin_bc.py "instancia"

# https://support.gurobi.com/hc/en-us/articles/360025804471-What-is-the-difference-between-user-cuts-and-lazy-constraints-
# https://www.gurobi.com/documentation/9.1/examples/cb_py.html
# https://www.linkedin.com/pulse/como-utilizar-fun%C3%A7%C3%A3o-callback-gurobi-igor-gir%C3%A3o
# https://www.gurobi.com/documentation/9.1/refman/py_model_cbcut.html
# https://www.gurobi.com/documentation/9.1/refman/cs_cb_addcut.html
# https://www.gurobi.com/documentation/9.1/refman/c_cbcut.html
# https://support.gurobi.com/hc/en-us/community/posts/360076323211-implement-cutting-plane-algorithm-
# https://www.gurobi.com/resource/mip-basics/
# https://support.gurobi.com/hc/en-us/articles/360040083172
# https://support.gurobi.com/hc/en-us/articles/360013197972
# https://www.gurobi.com/documentation/9.1/refman/cuts.html#parameter:Cuts
# https://www.gurobi.com/documentation/9.1/refman/precrush.html#parameter:PreCrush
# https://www.gurobi.com/documentation/9.1/refman/py_model_cbcut.html
# https://www.gurobi.com/documentation/9.1/refman/cb_codes.html#sec:CallbackCodes

def mycallback(model, where):
  
  if where == GRB.Callback.MIPNODE:
    # MIP node callback
    status = model.cbGet(GRB.Callback.MIPNODE_STATUS)
    print('**** New node ****')    
    if status == GRB.OPTIMAL:
      #print("status = ", status)
      x = model.cbGetNodeRel(model._vars)
      model.cbSetSolution(model.getVars(), x)
      #print("tam x:", len(x))
      #print(x)
      
#      print("N = ", N)
      k = N-1          
      for i in range(0,N):
        for j in range(i+1, N):
          k = k + 1 
          #print("i %d, j %d , k %d " %(i,j,k))
          if (x[i] + x[j] > 1 + x[k]):
            print("corte adicionado: x[%d] + x[%d] <= 1 + y[%d,%d]" %(i,j,i,j))
            model.cbCut(model._vars[i] + model._vars[j] <= 1 + model._vars[k])

def knapsack(datafile):

  with open(datafile, 'r') as file: linhas = file.readlines()

  linhas = [a.strip() for a in linhas] # remove linha vazia inicial e elimina os "\n" de cada linha

  n = int(linhas[0]) # ler o tamanho da instancia
  
  d = np.fromstring(linhas[1], dtype=int, sep = ' ') # ler a diagonal da matriz
  
  p = np.zeros((n), dtype=int) # define o vetor de lucro

  for i in range(n): # preenche a diagonal
    p[i] = d[i]

  c = int(linhas[2]) # ler a capacidade

  w = np.fromstring(linhas[3], dtype=int, sep = ' ') # ler os pesos

  print("N = ", n)

  model = gp.Model("knapsack") #cria o modelo

  x = model.addVars(n, vtype=GRB.BINARY, name='x') #Adicionando Variáveis 

  obj = None
  for i in range(0, n):
    obj += p[i] * x[i]
 
  model.setObjective(obj, GRB.MAXIMIZE)

  constr0 = None
  for j in range(0, n):
    constr0 += (w[j] * x[j])
  model.addConstr(constr0 <= c)

#  model.write("knapsack.lp")

  # configurando parametros
  #model.Params.IterationLimit = 1000 # define o número de iterações do simplex
  #model.Params.TimeLimit = 60 # define tempo limite
  model.Params.MIPGap = 0.001 # define valor do gap
  #model.Params.method = 1
  #model.Params.NodeMethod = -1 #  -1=automatic, 0=primal simplex, 1=dual simplex, and 2=barrier
  model.Params.Threads = 1
#  model.Params.Presolve = 0
#  model.Params.Cuts = 0
#  model.Params.PreCrush = 1

  # Turn off display and heuristics
  #gp.setParam('OutputFlag', 0)
  #gp.setParam('Heuristics', 0)

  # Open log file
#  logfile = open('cb.log', 'w')
 
  # Pass data into my callback function
  model._lastiter = -GRB.INFINITY
  model._lastnode = -GRB.INFINITY
#  model._logfile = logfile
  model._vars = model.getVars()

#  model.optimize(mycallback)
  model.optimize()


  #print('Obj: %g' % obj.getValue())
  
  print('')
  print('Optimization complete')
  if model.SolCount == 0:
    print('No solution found, optimization status = %d' % model.Status)
  else:
    print('Solution found, objective = %g' % model.ObjVal)
#    for v in model.getVars():
#      if v.X != 0.0:
#        print('%s %g' % (v.VarName, v.X))

  print('solução viável: ', round(model.objVal,2))
  print('bound: ', round(model.objBound,2))
  print('tempo: ', round(model.Runtime,2))
  print('gap: ', round(model.MIPGap,2))
  print('iterações: ', round(model.IterCount,2))
  print('Número de vértices: ', round(model.NodeCount,2))

if __name__ == "__main__":
  datafile = "instances/kp/10/10_100_1.txt"

  if len(sys.argv) < 2:
    print("Default data file : " + datafile)
  else:
    datafile = sys.argv[1]

  knapsack(datafile)
