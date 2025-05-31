import pandas as pd
import numpy as np

# “kleiner-ist-besser” S/N-Ratio
def sn_ratio(y):
    return -10 * np.log10((y**2).mean())

def load_csv(path):
    # Lies die CSV ein, behandle Komma als Dezimaltrenner
    return pd.read_csv(path, dtype=str)

def preprocess(df, numeric_cols):
    # Spaltennamen säubern
    df.columns = [c.strip().lower() for c in df.columns]
    # Komma-Dezimal in float umwandeln
    for col in numeric_cols:
        if col in df.columns:
            df[col] = (
                df[col]
                  .str.replace('.', '',  regex=False)  # Tausenderpunkte entfernen
                  .str.replace(',', '.', regex=False)  # Komma→Punkt
                  .astype(float)
            )
    return df

def analyze(csv_path, factors):
    df = load_csv(csv_path)
    # Alle Faktoren + fitness vormerken
    df = preprocess(df, factors + ['fitness'])
    print(f"\n=== Analyse für {csv_path} ===")
    for factor in factors:
        means = df.groupby(factor)['fitness'].mean()
        sn    = df.groupby(factor)['fitness'].apply(sn_ratio)
        delta = abs(means.iloc[1] - means.iloc[0])
        print(f"\nFaktor: {factor}")
        print("  Mittel-Fitness:   ", means.to_dict())
        print("  Mittel-S/N-Ratio: ", sn.to_dict())
        print("  Δ-Fitness:        ", delta)

if __name__ == "__main__":
    # 1) Codon-Taguchi (.csv)
    # analyze(
    #     csv_path="taguchi_L18_codon.csv",
    #     factors=[
    #         'popsize',
    #         'maxdepth',
    #         'wraps',
    #         'tournament',
    #         'cxprob',
    #         'mutation'
    #     ]
    # )

    # 2) Subtree-Taguchi (.csv)
    analyze(
        csv_path="taguchi_L18_Finetuning_subtree.csv",
        factors=[
            'popsize',
            'maxdepth',
            'wraps',
            'tournament',
            'cxprob'
        ]
    )
