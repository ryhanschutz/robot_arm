import trimesh, glob, os

pasta = r"C:\Users\ryhan_Schutz.EDU_FIESC\Documents\PlatformIO\Projects\robot_arm\simulation\assets"
arquivos = sorted(glob.glob(os.path.join(pasta, "*.stl")))

if not arquivos:
    print("Nenhum STL encontrado em:", pasta)

for f in arquivos:
    m = trimesh.load(f)
    c = m.bounds.mean(axis=0)
    print(f"{os.path.basename(f):35s} centro (mm): {c.round(1)}  tamanho: {m.extents.round(1)}")