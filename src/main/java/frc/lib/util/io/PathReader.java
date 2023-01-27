package frc.lib.util.io;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import edu.wpi.first.math.geometry.Rotation2d;
import org.ejml.simple.SimpleMatrix;
import frc.lib.util.control.Path;
import frc.lib.util.control.PathSegment;
import frc.lib.util.io.json.InterpolatingDoubleJsonHandler;
import frc.lib.util.io.json.PathSegmentJsonHandler;
import frc.lib.util.io.json.Rotation2JsonHandler;
import frc.lib.util.io.json.SimpleMatrixJsonHandler;
import frc.lib.util.util.InterpolatingDouble;

import java.io.IOException;
import java.io.Reader;
import java.lang.reflect.Type;
import java.util.Map;
import java.util.TreeMap;

public final class PathReader implements AutoCloseable {
    private final Gson gson;
    private final Reader in;

    public PathReader(Reader in) {
        this.gson = new GsonBuilder()
                .registerTypeAdapter(InterpolatingDouble.class, new InterpolatingDoubleJsonHandler())
                .registerTypeHierarchyAdapter(PathSegment.class, new PathSegmentJsonHandler())
                .registerTypeAdapter(Rotation2d.class, new Rotation2JsonHandler())
                .registerTypeAdapter(SimpleMatrix.class, new SimpleMatrixJsonHandler())
                .create();
        this.in = in;
    }

    public Path read() throws IOException {
        try {
            JsonElement rootElement = JsonParser.parseReader(in);
            if (!rootElement.isJsonObject()) {
                throw new IOException("Path must be a JSON object");
            }

            JsonObject root = rootElement.getAsJsonObject();
            if (!root.has("segments") || !root.has("rotations")) {
                throw new IOException("Path is not valid");
            }

            PathSegment[] pathSegments = gson.fromJson(root.get("segments"), PathSegment[].class);

            Type rotationMapType = new TypeToken<TreeMap<Double, Rotation2d>>() {
            }.getType();
            Map<Double, Rotation2d> rotations = gson.fromJson(root.get("rotations"), rotationMapType);

            return new Path(pathSegments, rotations);
        } catch (JsonIOException | JsonSyntaxException e) {
            throw new IOException(e);
        }
    }

    @Override
    public void close() throws IOException {
        in.close();
    }
}
