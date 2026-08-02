import EvoEngine.ShaderMigration.Leaf;

public interface IMigrationTransform
{
    float apply(float value);
}

public float migrationCommon(float value)
{
    return migrationLeaf(value);
}

public float migrationApply<T : IMigrationTransform>(T transform, float value)
{
    return transform.apply(migrationLeaf(value));
}
