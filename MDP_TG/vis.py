import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import Polygon

import numpy as np
import scipy.stats as stats
import random

matplotlib.rcParams['ps.useafm'] = True
matplotlib.rcParams['pdf.use14corefonts'] = True
matplotlib.rcParams['text.usetex'] = True


def visualize_run(XX, LL, UU, MM, name=None):
    # -----plot the sequence of states for the test run
    figure = plt.figure()
    ax = figure.add_subplot(1, 1, 1)
    N = len(XX)
    # print 'N: %s' %N
    # ----
    for n in range(0, N):
        X = list(XX[n])
        L = list(LL[n])
        U = list(UU[n])
        M = list(MM[n])
        K = len(X)
        # print 'K: %s' %K
        RAD = 0.3
        for k in range(0, K):
            if M[k] == 0:
                color = 'blue'
            if M[k] == 1:
                color = 'magenta'
            if M[k] == 2:
                color = 'black'
            # ----
            rec = matplotlib.patches.Rectangle((4*k-RAD, 3*n-RAD),
                                               RAD*2, RAD*2,
                                               fill=False,
                                               edgecolor=color,
                                               linewidth=3,
                                               ls='solid',
                                               alpha=1)
            ax.add_patch(rec)
            setstr = r''
            for s in L[k]:
                setstr += s
                setstr += ','
            ax.text(4*k-RAD, 3*n+RAD*4, r'$(%s, \{%s\})$' %
                    (str(X[k]), str(setstr)), fontsize=6, fontweight='bold')
            # ----
            if (k <= K-2):
                line = matplotlib.lines.Line2D([4*k+RAD, 4*k+4-RAD],
                                               [3*n, 3*n],
                                               linestyle='-',
                                               linewidth=1,
                                               color='black')
                ax.add_line(line)
                actstr = r''
                for s in U[k]:
                    actstr += s
                ax.text(4*k+2, 3*n+RAD, r'%s' %
                        str(actstr), fontsize=6, fontweight='bold')
    ax.set_aspect(0.7)
    ax.set_xlim(-1, 4*K)
    ax.set_ylim(-1, 3*N)
    ax.set_xlabel(r'$state\; sequence$')
    ax.set_ylabel(r'$run$')
    # ax.axis('off')
    if name:
        plt.savefig('%s.pdf' % name, bbox_inches='tight')
    return figure


def visualize_cost_vertical(UU, RD_UU, COST, name=None):
    # ----plot the distribution of total cost of all runs----
    # ----vertical plot
    figure = plt.figure()
    ax = figure.add_subplot(1, 1, 1)
    #
    cost_U = []
    for U in UU:
        c = 0
        for u in U:
            c += COST[u]
        cost_U.append(c)
    #
    cost_RD_U = []
    for U in RD_UU:
        c = 0
        for u in U:
            c += COST[u]
        cost_RD_U.append(c)
    #
    N = len(UU)
    unique_cost_U = list(set(cost_U))
    unique_cost_RD_U = list(set(cost_RD_U))
    ax.plot([1, ]*len(unique_cost_U), list(set(unique_cost_U)), 'ro')
    ax.plot([2, ]*len(unique_cost_RD_U), list(set(unique_cost_RD_U)), 'bo')
    ax.set_xlim(0, 3)
    ax.set_ylim(0, max([max(cost_U), max(cost_RD_U)])+2)
    ax.set_ylabel(r'$Total\; cost$')
    plt.xticks((1, 2), ('Optimized Strategy', 'Round-Robin'))
    if name:
        plt.savefig('%s.pdf' % name, bbox_inches='tight')
    return figure


def visualize_total_cost_horizontal(UU, RD_UU, COST, name=None):
    # ----plot the distribution of total cost for all runs----
    # ----horizontal plot
    figure = plt.figure()
    #
    cost_U = []
    for U in UU:
        c = 0
        for u in U:
            c += COST[u]
        cost_U.append(c)
    #
    cost_RD_U = []
    for U in RD_UU:
        c = 0
        for u in U:
            c += COST[u]
        cost_RD_U.append(c)
    #
    ax1 = figure.add_subplot(1, 2, 1)
    sorted_cost_U = sorted(cost_U)
    fit_U = stats.norm.pdf(sorted_cost_U, np.mean(
        sorted_cost_U), np.std(sorted_cost_U))
    ax1.plot(sorted_cost_U, fit_U, '-ro', label='Optimized Strategy')
    ax1.hist(sorted_cost_U, normed=True, color='green')
    ax1.set_xlabel(r'$Total\; cost$')
    ax1.set_ylabel(r'$Distribution$')
    ax1.legend(loc='upper right')
    #
    ax2 = figure.add_subplot(1, 2, 2)
    sorted_cost_RD_U = sorted(cost_RD_U)
    fit_RD_U = stats.norm.pdf(sorted_cost_RD_U, np.mean(
        sorted_cost_RD_U), np.std(sorted_cost_RD_U))
    ax2.plot(sorted_cost_RD_U, fit_RD_U, '-bo', label='Round-Robin')
    ax2.hist(sorted_cost_RD_U, normed=True, color='green')
    ax2.set_xlabel(r'$Total\; cost$')
    ax2.legend(loc='upper right')
    if name:
        plt.savefig('%s.pdf' % name, bbox_inches='tight')
    return figure


def visualize_suffix_cost_horizontal(UU, MM, RD_UU, RD_MM, COST, name=None):
    # ----plot the distribution of only suffix cost (accepting cyclic paths) for all runs----
    figure = plt.figure()
    # record cost of accepting cyclic path
    cost_U = []
    for k, U in enumerate(UU):
        M = MM[k]
        c = 0
        for j, u in enumerate(U):
            if M[j] == 1:
                c += COST[u]
            elif M[j] == 10:
                cost_U.append(c)
                c = 0
    # record cost of accepting cyclic path
    cost_RD_U = []
    for k, RD_U in enumerate(RD_UU):
        RD_M = RD_MM[k]
        c = 0
        for j, u in enumerate(RD_U):
            if RD_M[j] == 1:
                c += COST[u]
            elif RD_M[j] == 10:
                cost_RD_U.append(c)
                c = 0
    #
    ax1 = figure.add_subplot(1, 2, 1)
    sorted_cost_U = sorted(cost_U)
    fit_U = stats.norm.pdf(sorted_cost_U, np.mean(
        sorted_cost_U), np.std(sorted_cost_U))
    ax1.plot(sorted_cost_U, fit_U, '-ro', label='Optimized : Strategy')
    ax1.hist(sorted_cost_U, normed=True, color='green')
    ax1.set_xlabel(r'$Suffix\; cost$')
    ax1.set_ylabel(r'$Distribution$')
    ax1.legend(loc='upper right')
    #
    ax2 = figure.add_subplot(1, 2, 2)
    sorted_cost_RD_U = sorted(cost_RD_U)
    fit_RD_U = stats.norm.pdf(sorted_cost_RD_U, np.mean(
        sorted_cost_RD_U), np.std(sorted_cost_RD_U))
    ax2.plot(sorted_cost_RD_U, fit_RD_U, '-bo', label='Round-Robin')
    ax2.hist(sorted_cost_RD_U, normed=True, color='green')
    ax2.set_xlabel(r'$Suffix\; cost$')
    ax2.legend(loc='best')
    if name:
        plt.savefig('%s.pdf' % name, bbox_inches='tight')
    return figure


def analyze_events(MM, LL):
    # ----analyze the results of all runs----
    # prefix failure/success, suffix failure/success
    N = len(LL)
    T = len(LL[0])
    failure_count = 0.0
    prefix_failure_count = 0.0
    suffix_failure_count = 0.0
    prefix_suc_count = 0.0
    suffix_suc_count = 0.0
    for k, L in enumerate(LL):
        M = MM[k]
        Failure = False
        Prefix_suc = False
        Suffix_suc = False
        ip_count = 0
        for n, l in enumerate(L):
            m = M[n]
            if (m == 2) and (not Suffix_suc):
                Failure = True
                failure_count += 1
                if not Prefix_suc:
                    prefix_failure_count += 1
                else:
                    suffix_failure_count += 1
                break
            if (m == 1) and (not Prefix_suc):
                Prefix_suc = True
                prefix_suc_count += 1
            if m == 10:
                ip_count += 1
            if ip_count >= 1:
                Suffix_suc = True
                suffix_suc_count += 1
                break
    print('Analyze done')
    print('Total %s simulations: %s failure (%s) (%s prefix, %s suffix), %s prefix successful (%s); %s suffix successful (%s)' % (str(N), str(failure_count), str(failure_count/N),
          str(prefix_failure_count), str(suffix_failure_count), str(prefix_suc_count), str(prefix_suc_count/N), str(suffix_suc_count), str(suffix_suc_count/prefix_suc_count)))


def visualize_world(WS_d, WS_node_dict, name=None):
    # ----visualization for square-partitioned-workspace----
    # ----see case_study.py for details----
    figure = plt.figure()
    ax = figure.add_subplot(1, 1, 1)
    # ----- draw the workspace
    for node, prop in WS_node_dict.items():
        if frozenset(['base1', 'base']) in list(prop.keys()):
            text = '$Base1$'
            color = 'yellow'
        elif frozenset(['base2', 'base']) in list(prop.keys()):
            text = '$Base2$'
            color = 'yellow'
        elif frozenset(['base3', 'base']) in list(prop.keys()):
            text = '$Base3$'
            color = 'yellow'
        elif frozenset(['obstacle', 'low']) in list(prop.keys()):
            text = '$Obs: 0.2$'
            color = '#ff8000'
        elif frozenset(['obstacle', 'top']) in list(prop.keys()):
            text = '$Obs: 1.0$'
            color = 'red'
        elif frozenset(['supply', ]) in list(prop.keys()):
            text = '$Sply: %s$' % str(prop[frozenset(['supply', ])])
            if prop[frozenset(['supply', ])] >= 0.8:
                color = '#0000ff'
            elif prop[frozenset(['supply', ])] >= 0.6:
                color = '#0040ff'
            elif prop[frozenset(['supply', ])] >= 0.4:
                color = '#0080ff'
            elif prop[frozenset(['supply', ])] >= 0.2:
                color = '#00bfff'
        else:
            text = None
            color = 'white'
        rec = matplotlib.patches.Rectangle((node[0]-WS_d, node[1]-WS_d),
                                           WS_d*2, WS_d*2,
                                           fill=True,
                                           facecolor=color,
                                           edgecolor='black',
                                           linewidth=1,
                                           ls='--',
                                           alpha=0.8)
        ax.add_patch(rec)
        if text:
            ax.text(node[0]-0.7, node[1], r'%s' %
                    text, fontsize=10, fontweight='bold')
    ax.set_aspect('equal')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_xlabel(r'$x(m)$')
    ax.set_ylabel(r'$y(m)$')
    if name:
        plt.savefig('%s.pdf' % name, bbox_inches='tight')
    return figure


def visualize_world_paths(WS_d, WS_node_dict, XX, LL, UU, MM, name=None):
    # ----visualize simulated runs----
    N = len(XX)
    max_x = min_x = 0
    max_y = min_y = 0
    # ----
    for n in range(0, N):
        figure = plt.figure()
        ax = figure.add_subplot(1, 1, 1)
        text1 = None
        text2 = None
        # ----- draw the workspace
        for node, prop in WS_node_dict.items():
            if frozenset(['base1', 'base']) in list(prop.keys()):
                text1 = '$\mathrm{bs}_1$'
                text2 = '1.0'
                color = 'yellow'
            elif frozenset(['base2', 'base']) in list(prop.keys()):
                text1 = '$\mathrm{bs}_2$'
                text2 = '1.0'
                color = 'yellow'
            elif frozenset(['base3', 'base']) in list(prop.keys()):
                text1 = '$\mathrm{bs}_3$'
                text2 = '1.0'
                color = 'yellow'
            elif frozenset(['obstacle', 'low']) in list(prop.keys()):
                text1 = '$\mathrm{obs}$'
                text2 = '0.1'
                color = '#ff7f7f'
            elif frozenset(['obstacle', 'middle']) in list(prop.keys()):
                text1 = '$\mathrm{obs}$'
                text2 = '0.5'
                color = '#ff8000'
            elif frozenset(['obstacle', 'top']) in list(prop.keys()):
                text1 = '$\mathrm{obs}$'
                text2 = '1.0'
                color = 'red'
            elif frozenset(['obstacle', 'top']) in list(prop.keys()):
                text1 = '$\mathrm{obs}$'
                text2 = '$1.0$'
                color = 'red'
            elif frozenset(['supply', ]) in list(prop.keys()):
                text1 = '$\mathrm{spl}$'
                text2 = '%s' % str(prop[frozenset(['supply', ])])
                if prop[frozenset(['supply', ])] >= 0.8:
                    color = '#0000ff'
                elif prop[frozenset(['supply', ])] >= 0.6:
                    color = '#0040ff'
                elif prop[frozenset(['supply', ])] >= 0.4:
                    color = '#0080ff'
                elif prop[frozenset(['supply', ])] >= 0.2:
                    color = '#00bfff'
            elif frozenset(['md', ]) in list(prop.keys()):
                text1 = '$\mathrm{md}$'
                text2 = '%s' % str(prop[frozenset(['md', ])])
                if prop[frozenset(['md', ])] >= 0.8:
                    color = '#0000ff'
                elif prop[frozenset(['md', ])] >= 0.6:
                    color = '#0040ff'
                elif prop[frozenset(['md', ])] >= 0.4:
                    color = '#0080ff'
                elif prop[frozenset(['md', ])] >= 0.2:
                    color = '#00bfff'
            elif frozenset(['stair', ]) in list(prop.keys()):
                text1 = '$\mathrm{str}$'
                text2 = '%s' % str(prop[frozenset(['stair', ])])
                if prop[frozenset(['stair', ])] >= 0.8:
                    color = '#006400'
                elif prop[frozenset(['stair', ])] >= 0.6:
                    color = '#228B22'
                elif prop[frozenset(['stair', ])] >= 0.4:
                    color = '#90EE90'
                elif prop[frozenset(['stair', ])] >= 0.2:
                    color = '#98FB98'
            else:
                text1 = None
                text2 = None
                color = 'white'
            rec = matplotlib.patches.Rectangle((node[0]-WS_d, node[1]-WS_d),
                                               WS_d*2, WS_d*2,
                                               fill=True,
                                               facecolor=color,
                                               edgecolor='black',
                                               linewidth=1,
                                               ls='--',
                                               alpha=0.8)
            ax.add_patch(rec)
            if text1:
                ax.text(node[0]-0.08, node[1]+0.03, r'%s' %
                        text1, fontsize=10, fontweight='bold')
            if text2:
                ax.text(node[0]-0.08, node[1]-0.15, r'%s' %
                        text2, fontsize=10, fontweight='bold')
            min_x = min([node[0]+WS_d, min_x])
            max_x = max([node[0]+WS_d, max_x])
            min_y = min([node[1]+WS_d, min_y])
            max_y = max([node[1]+WS_d, max_y])
        X = list(XX[n])
        L = list(LL[n])
        U = list(UU[n])
        M = list(MM[n])
        K = len(X)
        # print 'K: %s' %K
        for k in range(0, K):
            if M[k] == 0:
                Ecolor = 'blue'
            if M[k] == 1:
                Ecolor = 'magenta'
            if M[k] == 2:
                Ecolor = 'black'
            if M[k] > 2:
                Ecolor = 'magenta'
            # ----
            #if (M[k] == 1) and (k<= K-2):
            if (k <= K-2):
                line = matplotlib.lines.Line2D([X[k][0], X[k+1][0]],
                                               [X[k][1], X[k+1][1]],
                                               linestyle='-',
                                               linewidth=1,
                                               color=Ecolor)
                ax.add_line(line)
                xl = X[k][0]
                yl = X[k][1]
                dl = X[k][2]
                car_size = 0.1
                if dl == 'N':
                    car = [(xl-car_size, yl-car_size), (xl-car_size, yl+car_size),
                           (xl, yl+2*car_size), (xl+car_size, yl+car_size), (xl+car_size, yl-car_size)]
                if dl == 'E':
                    car = [(xl-car_size, yl+car_size), (xl+car_size, yl+car_size),
                           (xl+2*car_size, yl), (xl+car_size, yl-car_size), (xl-car_size, yl-car_size)]
                if dl == 'S':
                    car = [(xl+car_size, yl+car_size), (xl+car_size, yl-car_size),
                           (xl, yl-2*car_size), (xl-car_size, yl-car_size), (xl-car_size, yl+car_size)]
                if dl == 'W':
                    car = [(xl+car_size, yl-car_size), (xl-car_size, yl-car_size),
                           (xl-2*car_size, yl), (xl-car_size, yl+car_size), (xl+car_size, yl+car_size)]
                polygon = Polygon(car, facecolor='black',
                                  edgecolor='black', lw=0.5, alpha=0.7)
                ax.add_patch(polygon)
                #
                # actstr = r''
                # for s in U[k]:
                #     actstr += s
                # ax.text(xl, yl+0.7, r'%s' %str(actstr), fontsize = 15, fontweight = 'bold', color='red')
        ax.set_aspect('equal')
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_xlabel(r'$x(m)$')
        ax.set_ylabel(r'$y(m)$')
        if name:
            plt.savefig('%s%s.pdf' % (name, str(n)), bbox_inches='tight')


def visualize_state_dynamic(motion_mdp, WS_d, WS_node_dict, x, l, u, m):
    # ----plot the dynamic workspace and robot motion for each time step
    figure = plt.figure()
    ax = figure.add_subplot(1, 1, 1)
    # ----- draw the workspace
    xl = x[0]
    yl = x[1]
    dl = x[2]
    if m == 0:
        Ecolor = 'green'
    if m == 1:
        Ecolor = 'magenta'
    if m == 2:
        Ecolor = 'black'
    if m > 2:
        Ecolor = 'magenta'
    if dl == 'N':
        car = [(xl-0.2, yl-0.2), (xl-0.2, yl+0.2), (xl, yl+0.4),
               (xl+0.2, yl+0.2), (xl+0.2, yl-0.2)]
    if dl == 'E':
        car = [(xl-0.2, yl+0.2), (xl+0.2, yl+0.2), (xl+0.4, yl),
               (xl+0.2, yl-0.2), (xl-0.2, yl-0.2)]
    if dl == 'S':
        car = [(xl+0.2, yl+0.2), (xl+0.2, yl-0.2), (xl, yl-0.4),
               (xl-0.2, yl-0.2), (xl-0.2, yl+0.2)]
    if dl == 'W':
        car = [(xl+0.2, yl-0.2), (xl-0.2, yl-0.2), (xl-0.4, yl),
               (xl-0.2, yl+0.2), (xl+0.2, yl+0.2)]
    polygon = Polygon(car, fill=True, facecolor=Ecolor,
                      edgecolor=Ecolor, lw=5, zorder=2)
    ax.add_patch(polygon)
    # ----------------
    for node, prop in WS_node_dict.items():
        if node != (xl, yl):
            S = []
            P = []
            for s, p in prop.items():
                S.append(s)
                P.append(p)
            rdn = random.random()
            pc = 0
            for k, p in enumerate(P):
                pc += p
                if pc > rdn:
                    break
            current_s = S[k]
        if node == (xl, yl):
            current_s = l
        # ------
        if current_s == frozenset(['base1', 'base']):
            text = '$base1$'
            color = 'yellow'
        elif current_s == frozenset(['base2', 'base']):
            text = '$base2$'
            color = 'yellow'
        elif current_s == frozenset(['base3', 'base']):
            text = '$base3$'
            color = 'yellow'
        elif current_s == frozenset(['obstacle', 'low']):
            text = '$Obs$'
            color = '#ff8000'
        elif current_s == frozenset(['obstacle', 'top']):
            text = '$Obs$'
            color = 'red'
        elif current_s == frozenset(['supply', ]):
            text = '$Sply$'
            color = '#0000ff'
        else:
            text = None
            color = 'white'
        rec = matplotlib.patches.Rectangle((node[0]-WS_d, node[1]-WS_d),
                                           WS_d*2, WS_d*2,
                                           fill=True,
                                           facecolor=color,
                                           edgecolor='black',
                                           linewidth=1,
                                           ls='--',
                                           alpha=0.8)
        ax.add_patch(rec)
        if text:
            ax.text(node[0]-0.7, node[1], r'%s' %
                    text, fontsize=10, fontweight='bold')
    ax.set_aspect('equal')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_xlabel(r'$x(m)$')
    ax.set_ylabel(r'$y(m)$')
    return figure


def visualize_state_action_dynamic(motion_mdp, WS_d, WS_node_dict, x, l, u, m):
    # ----visualize dynamic workspace and robot motion at each step----
    # ----with action name and possible post states----
    figure = plt.figure()
    ax = figure.add_subplot(1, 1, 1)
    # ----- draw the workspace
    xl = x[0]
    yl = x[1]
    dl = x[2]
    if m == 0:
        Ecolor = 'green'
    if m == 1:
        Ecolor = 'magenta'
    if m == 2:
        Ecolor = 'black'
    if m > 2:
        Ecolor = 'magenta'
    if dl == 'N':
        car = [(xl-0.2, yl-0.2), (xl-0.2, yl+0.2), (xl, yl+0.4),
               (xl+0.2, yl+0.2), (xl+0.2, yl-0.2)]
    if dl == 'E':
        car = [(xl-0.2, yl+0.2), (xl+0.2, yl+0.2), (xl+0.4, yl),
               (xl+0.2, yl-0.2), (xl-0.2, yl-0.2)]
    if dl == 'S':
        car = [(xl+0.2, yl+0.2), (xl+0.2, yl-0.2), (xl, yl-0.4),
               (xl-0.2, yl-0.2), (xl-0.2, yl+0.2)]
    if dl == 'W':
        car = [(xl+0.2, yl-0.2), (xl-0.2, yl-0.2), (xl-0.4, yl),
               (xl-0.2, yl+0.2), (xl+0.2, yl+0.2)]
    polygon = Polygon(car, fill=True, facecolor=Ecolor,
                      edgecolor=Ecolor, lw=5, zorder=2)
    ax.add_patch(polygon)
    #
    actstr = r''
    for s in u:
        actstr += s
    ax.text(xl, yl+0.5, r'$%s$' % str(actstr),
            fontsize=13, fontweight='bold', color='red')
    # plot shadow
    t_x_list = []
    for t_x in motion_mdp.successors_iter(x):
        prop = motion_mdp[x][t_x]['prop']
        if u in list(prop.keys()):
            t_x_list.append((t_x, prop[u][0]))
    #
    for new_x in t_x_list:
        xl = new_x[0][0]
        yl = new_x[0][1]
        dl = new_x[0][2]
        if dl == 'N':
            car = [(xl-0.2, yl-0.2), (xl-0.2, yl+0.2), (xl, yl+0.4),
                   (xl+0.2, yl+0.2), (xl+0.2, yl-0.2)]
        elif dl == 'E':
            car = [(xl-0.2, yl+0.2), (xl+0.2, yl+0.2), (xl+0.4, yl),
                   (xl+0.2, yl-0.2), (xl-0.2, yl-0.2)]
        elif dl == 'S':
            car = [(xl+0.2, yl+0.2), (xl+0.2, yl-0.2), (xl, yl-0.4),
                   (xl-0.2, yl-0.2), (xl-0.2, yl+0.2)]
        elif dl == 'W':
            car = [(xl+0.2, yl-0.2), (xl-0.2, yl-0.2), (xl-0.4, yl),
                   (xl-0.2, yl+0.2), (xl+0.2, yl+0.2)]
        polygon = Polygon(car, fill=True, facecolor='grey',
                          edgecolor='grey', lw=5, zorder=1)
        ax.add_patch(polygon)
        prob = new_x[1]
        ax.text(xl, yl, r'$%s$' % str(prob), fontsize=10,
                fontweight='bold', color='red')
    # ----------------
    for node, prop in WS_node_dict.items():
        if node != (x[0], x[1]):
            S = []
            P = []
            for s, p in prop.items():
                S.append(s)
                P.append(p)
            rdn = random.random()
            pc = 0
            for k, p in enumerate(P):
                pc += p
                if pc > rdn:
                    break
            current_s = S[k]
        if node == (x[0], x[1]):
            current_s = l
        # ------
        if current_s == frozenset(['base1', 'base']):
            text = '$base1$'
            color = 'yellow'
        elif current_s == frozenset(['base2', 'base']):
            text = '$base2$'
            color = 'yellow'
        elif current_s == frozenset(['base3', 'base']):
            text = '$base3$'
            color = 'yellow'
        elif current_s == frozenset(['obstacle', 'low']):
            text = '$Obs$'
            color = '#ff8000'
        elif current_s == frozenset(['obstacle', 'top']):
            text = '$Obs$'
            color = 'red'
        elif current_s == frozenset(['supply', ]):
            text = '$Sply$'
            color = '#0000ff'
        else:
            text = None
            color = 'white'
        rec = matplotlib.patches.Rectangle((node[0]-WS_d, node[1]-WS_d),
                                           WS_d*2, WS_d*2,
                                           fill=True,
                                           facecolor=color,
                                           edgecolor='black',
                                           linewidth=1,
                                           ls='--',
                                           alpha=0.8)
        ax.add_patch(rec)
        if text:
            ax.text(node[0]-0.7, node[1], r'%s' %
                    text, fontsize=10, fontweight='bold')
    ax.set_aspect('equal')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 10)
    ax.set_xlabel(r'$x(m)$')
    ax.set_ylabel(r'$y(m)$')
    return figure


def run_movie(motion_mdp, WS_d, WS_node_dict, X, L, U, M):
    # ----save simulation plot at each time step----
    # ----for movie compilation, see mkmovie.sh for details----
    DPI = 500
    i = 0
    figure1 = visualize_state_action_dynamic(
        motion_mdp, WS_d, WS_node_dict, X[0], L[0], U[0], M[0])
    figure1.savefig('movie/frame%s.png' % i, dpi=DPI)
    plt.close()
    i += 1
    for k in range(1, len(X)):
        figure2 = visualize_state_dynamic(
            motion_mdp, WS_d, WS_node_dict, X[k], L[k], U[k], M[k])
        figure2.savefig('movie/frame%s.png' % i, dpi=DPI)
        plt.close()
        i += 1
        figure1 = visualize_state_action_dynamic(
            motion_mdp, WS_d, WS_node_dict, X[k], L[k], U[k], M[k])
        figure1.savefig('movie/frame%s.png' % i, dpi=DPI)
        plt.close()
        i += 1
        plt.close()


def compute_suffix_mean_cost(UU, MM, COST):
    # record mean total cost of accepting cyclic path
    mean_cost_U = []
    for k, U in enumerate(UU):
        M = MM[k]
        c = 0.0
        t = 0.0
        for j, u in enumerate(U):
            if M[j] == 1:
                c += COST[u]
                t += 1.0
            elif M[j] == 10:
                mean_cost_U.append(c/t)
                c = 0.0
                t = 0.0
    print('total number of cyclic path:%d' % len(mean_cost_U))
    mean_cost = sum(mean_cost_U)/len(mean_cost_U)
    return mean_cost
