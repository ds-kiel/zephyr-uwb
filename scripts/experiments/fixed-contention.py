import math
import sys

from scipy import stats
import scipy
import numpy as np

import pandas as pd

import json

import matplotlib.pyplot as plt
import seaborn as sns


from scipy import stats
from sklearn.feature_selection import mutual_info_classif
from sklearn.ensemble import RandomForestClassifier
from sklearn.inspection import permutation_importance
from sklearn.decomposition import PCA
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
from sklearn import tree


# read in json file with tx and rx events from path
def read_json_lines(file_path):
    # Read the file line by line and parse each JSON object
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            try:
                parsed_json = json.loads(line)

                # if event is rx or tx add to table
                if parsed_json['event'] == 'rx' or parsed_json['event'] == 'tx':
                    data.append(parsed_json)

            except json.JSONDecodeError:
                print('Error decoding JSON:', line)

    # Create a DataFrame from the list of dictionaries
    df = pd.DataFrame(data)

    df = df.sort_values(['asn'])

    return df


def read_json_distances(file_path):
    # read in pandas json gziped dataframe
    df = pd.read_json(file_path, compression='gzip', lines=True)
    return df


def find_concurrent_transmission(df):
    # return all rows with  event == 'tx' and where the count of (slot phase and asn) is more than 1
    tx_df = df[df['event'] == 'tx']
    mask = tx_df.groupby(['asn', 'slot', 'phase']).count()['event'] > 1
    return tx_df[tx_df.set_index(['asn', 'slot', 'phase']).index.isin(mask[mask].index)]


def fit_random_forest(X, y, features):
    # Random Forest Feature Importance
    rf = RandomForestClassifier(n_estimators=2, random_state=42)
    estimator = rf.fit(X, y)
    rf_feature_importance = pd.Series(rf.feature_importances_, index=features).sort_values(ascending=False)

    print('\nRandom Forest Feature Importance:')
    print('Accuracy:', rf.score(X, y))
    print(rf_feature_importance)

    return estimator


def fit_decision_tree(X, y, features):
    dt = tree.DecisionTreeClassifier()
    estimator = dt.fit(X, y)

    print('\nDecision Tree Feature Importance:')
    print('Accuracy:', dt.score(X, y))
    print(pd.Series(dt.feature_importances_, index=features).sort_values(ascending=False))

    return estimator


def create_confusion_matrix(y_test, y_pred_test):
    # Get and reshape confusion matrix data
    matrix = confusion_matrix(y_test, y_pred_test)
    matrix = matrix.astype('float') / matrix.sum(axis=1)[:, np.newaxis]

    # Build the plot
    plt.figure(figsize=(16, 7))
    sns.set(font_scale=1.4)
    sns.heatmap(matrix, annot=True, annot_kws={'size': 10},
                cmap=plt.cm.Greens, linewidths=0.2)

    # Add labels to the plot
    class_names = ['concurrent', 'non-concurrent']
    tick_marks = np.arange(len(class_names))
    tick_marks2 = tick_marks + 0.5
    plt.xticks(tick_marks, class_names, rotation=25)
    plt.yticks(tick_marks2, class_names, rotation=0)
    plt.xlabel('Predicted label')
    plt.ylabel('True label')
    plt.title('Confusion Matrix for Random Forest Model')
    plt.show()


if __name__ == '__main__':
    # testbed_name = sys.argv[1]
    # file_path = sys.argv[2]

    testbed_name = 'ds_kiel'
    file_path = '/Users/christian/Promotion/ScalableRanging/zephyr-uwb/scripts/data/testbed/kiel/fixed-contention/experiment.log'
    dist_path = '/Users/christian/Promotion/ScalableRanging/zephyr-uwb/scripts/data/testbed/kiel/fixed-contention/distances.json.gz'

    df = read_json_lines(file_path)
    dist_df = read_json_distances(dist_path)

    con_tx_df = find_concurrent_transmission(df)
    con_tx_locs_df = con_tx_df[['asn', 'slot', 'phase']].drop_duplicates()

    # retrieve rx rows with same asn, slot, phase as in con_tx_locs_df
    con_rx_df = df[(df['event'] == 'rx') & df.set_index(['asn', 'slot', 'phase'])
                   .index.isin(con_tx_locs_df.set_index(['asn', 'slot', 'phase']).index)]
    # retrieve all other rx rows not in con_rx_df
    non_con_rx_df = df[(df['event'] == 'rx')
                       & ~df.set_index(['asn', 'slot', 'phase'])
                       .index.isin(con_tx_locs_df.set_index(['asn', 'slot', 'phase']).index)]

    # look at receptions of node with own_id 0
    own_id_0_rx = con_rx_df[con_rx_df['own_id'] == 0]
    # for each row we get a asn and other_id, add to own_id_0_rx the dsitance for own_id, other_id, asn from dist_df
    own_id_0_rx = dist_df.merge(own_id_0_rx, how='inner', on=['asn', 'own_id', 'other_id'])


    features = [
        'fp_index',
        'fp_ampl1', 'fp_ampl2', 'fp_ampl3',
        'cir_pwr', 'rx_pacc', 'std_noise'
    ]

    # split dataset into training and test set
    con_rx_df_train = con_rx_df.sample(frac=0.8, random_state=42)
    con_rx_df_test = con_rx_df.drop(con_rx_df_train.index)
    non_con_rx_df_train = non_con_rx_df.sample(frac=0.8, random_state=42)
    non_con_rx_df_test = non_con_rx_df.drop(non_con_rx_df_train.index)

    # Combine the datasets and create labels
    X_train = pd.concat([con_rx_df_train[features], non_con_rx_df_train[features]])
    X_train_conc_labels = pd.concat([con_rx_df_train['concurrent_senders'], non_con_rx_df_train['concurrent_senders']])
    y_train = np.concatenate([np.ones(len(con_rx_df_train)),
                              np.zeros(len(non_con_rx_df_train))])


    X_test = pd.concat([con_rx_df_test[features], non_con_rx_df_test[features]])
    y_test = np.concatenate([np.ones(len(con_rx_df_test)),
                             np.zeros(len(non_con_rx_df_test))])

    print(np.unique(y_train, return_counts=True))

    rf_estimator = fit_random_forest(X_train, y_train, features)
    dt_estimator = fit_decision_tree(X_train, y_train, features)

    # look at performance of rf_estimator on test set
    print('RF Accuracy on test set:', rf_estimator.score(X_test, y_test))
    print('DT Accuracy on test set:', dt_estimator.score(X_test, y_test))

    create_confusion_matrix(y_test, rf_estimator.predict(X_test))
    create_confusion_matrix(y_test, dt_estimator.predict(X_test))

    # Perform PCA to reduce to 2 dimensions
    pca = PCA(n_components=2)
    X_pca = pca.fit_transform(X_train)

    # Plot the PCA results
    plt.figure(figsize=(10, 7))
    scatter = plt.scatter(X_pca[:, 0], X_pca[:, 1],
                          c=X_train_conc_labels, cmap='viridis', alpha=0.7)

    plt.title('PCA of Concurrent and Non-Concurrent RX Events')
    plt.xlabel('Principal Component 1')
    plt.ylabel('Principal Component 2')
    plt.colorbar(scatter, label='Class')
    plt.show()


    # compare con_rx_df and non_con_rx_df by plotting the values of the columns in features
    for row in features:
        con_rx_values = con_rx_df[row].values
        non_con_rx_values = non_con_rx_df[row].values

        # don't use absolute values, but use relative frequency
        plt.hist(con_rx_values, bins=100, alpha=0.5, label='concurrent', density=True)
        plt.hist(non_con_rx_values, bins=100, alpha=0.5, label='non-concurrent', density=True)

        plt.legend(loc='upper right')
        plt.title(row)
        plt.show()


    # Display the first few rows of the DataFrame
    print(df.head())

    # Display basic information about the DataFrame
    print(df.info())

    print(df.describe())
