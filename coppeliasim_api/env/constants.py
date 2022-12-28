import pathlib

print("\033[0;34;47m"+"assurez vous que le dossier CoppeliaSim_Edu_V4_3_0_Ubuntu20_04 se trouve a la racine du projet\n\n\nUne erreur peux survenir si votre version du dossier Copeliasim n'est pas nommé CoppeliaSim_Edu_V4_3_0_Ubuntu20_04, renommez le en consequence "+'\033[0m')
COPSIM_DIR = "./CoppeliaSim_Edu_V4_3_0_Ubuntu20_04"

if __name__ == "__main__":
    print(f"COPSIM_DIR: {COPSIM_DIR}")
