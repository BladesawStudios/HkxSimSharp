using System;
using System.Collections.Generic;
using HkxSimSharp.Cloth.Data.Collidables;
using HkxSimSharp.Cloth.Data.Operators;

namespace HkxSimSharp.Cloth.Data;

public sealed class HclClothData
{
    public string Name { get; set; } = string.Empty;
    public List<HclSimClothData> SimClothDatas { get; } = new();
    public List<HclBufferDefinition> BufferDefinitions { get; } = new();
    public List<HclTransformSetDefinition> TransformSetDefinitions { get; } = new();
    public List<HclOperator> Operators { get; } = new();
    public uint TargetPlatform { get; set; }
}

public sealed class HclClothContainer
{
    public List<HclCollidable> Collidables { get; } = new();
    public List<HclClothData> ClothDatas { get; } = new();
}
